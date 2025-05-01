#define BLYNK_TEMPLATE_ID           "TMPxxxxxx"
#define BLYNK_TEMPLATE_NAME         "Device"
#define BLYNK_AUTH_TOKEN            "dyyqjCuZCtTym54uGsvdtFgmT7ZR_Hq2"
#define BLYNK_PRINT Serial
#define ANALOG_IN_PIN  33  // ESP32 pin GPIO36 (ADC0) connected to voltage sensor
#define R1             30000.0  // Resistor values in ohms
#define R2             7500.0   // Resistor values in ohms
#define NUM_SAMPLES    5   // Number of ADC samples for averaging
#define MAX485_DE_RE 4  // DE/RE pin connected to GPIO2 (D4)
#define ACS_PIN 32      // ACS712 connected to GPIO32
#define relaypin1  2 //v34
#define relaypin2  23 //v35
#define relaypin3  14 //v36
#define relaypin4  27 //v37
#define relaypin5  25 //v38
#define relaypin6  26 //v39
//#define BUZZER_PIN  23 //buzzer pin

#include <ArduinoOTA.h>
#include <esp_adc_cal.h>
#include <Arduino.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ACS712.h>
int powerstatus=0;
int inverterstatus=0;
int mirror1=0;
esp_adc_cal_characteristics_t *adc_chars;                       // Calibration structure
char ssid[] = "BlynkHotspot";
char pass[] = "sk12345678";
char auth[] = "dyyqjCuZCtTym54uGsvdtFgmT7ZR_Hq2";               // Blynk authentication token
char server[] = "10.42.0.1";                                    // Local Blynk server IP
int port = 8080;                                                // Blynk server port
TaskHandle_t Core0Task;
TaskHandle_t Core1Task;
BlynkTimer timer;
HardwareSerial pzemSerial(2);                                    // Use Serial2 (UART2)
ModbusMaster pzem1;                                              // Create Modbus instances
ModbusMaster pzem3;
const int THERMISTOR_PIN = 34; // Any ESP32 ADC pin
const float SERIES_RESISTOR = 10000;  // 10kΩ fixed resistor
const float NOMINAL_RESISTANCE = 7800; // 7.1kΩ at 25°C (adjust if needed)
const float NOMINAL_TEMPERATURE = 25.0; // 25°C reference
const float B_COEFFICIENT = 4100; // Beta coefficient (adjust if needed)
int acsReadings[NUM_SAMPLES] = {0}; // Array to store ACS712 readings
int acsReadIndex = 0;
int acsTotal = 0;
int acsAdvVal = 0;
int acsValAvrg = 0;
ACS712  ACS(32, 3.3, 4095, 66);
// Global variables for sharing data between cores
volatile float shared_voltage_in = 0;
volatile float shared_voltage_adc = 0;
volatile int shared_acsAdvVal = 0;
volatile int shared_acsValAvrg = 0;
int temp=0;
int core0delay=1000;
int core1delay=1000;
int core2delay=1000;
int buzzeractive=1;
int bhi=200;
int blo=100;
int brp=6;
int alarm1=0;
int overv=250;
int underv=170;
int overload=3500;
int overheat=65;
int BUZZER_PIN = 23; //buzzer pin

void beep(int highTime, int lowTime, int repeat = 1) {
  for (int i = 0; i < repeat; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(highTime);
    digitalWrite(BUZZER_PIN, LOW);
    delay(lowTime);
  }
}

void playAlertCode(int code) {
  switch (code) {
    case 1: // Power Up
      beep(200, 0);
      break;

    case 2: // Power Down
      beep(200, 100);
      beep(200, 0);
      break;

    case 3: // High Temp
      beep(400, 100, 3);
      break;

    case 4: // High Voltage
      beep(400, 100, 2);
      beep(400, 200, 2);
      break;

    case 5: // Overload High
      beep(500, 100, 5);
      break;

    case 6: // Low Voltage
      beep(400, 200, 2);
      beep(200, 100, 2);
      beep(100, 50, 2);
      break;
    default:
      // Unknown code
      break;
  }
}
BLYNK_WRITE(v31) //speed control 0
{
  core0delay = param.asInt();
}
BLYNK_WRITE(V32) //speed control 1
{
  core1delay= param.asInt();
}
BLYNK_WRITE(v33) //speed control loop
{
  core2delay= param.asInt();
}

BLYNK_WRITE(V34) //relay 1
{
  if(param.asInt()==1){
    digitalWrite(relaypin1, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin1, 0);
  }
}
BLYNK_WRITE(V35) //relay 2
{
  if(param.asInt()==1){
    digitalWrite(relaypin2, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin2, 0);
  }
}
BLYNK_WRITE(V36) //relay 3
{
  if(param.asInt()==1){
    digitalWrite(relaypin3, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin3, 0);
  }
}
BLYNK_WRITE(V37) //relay 4
{
  if(param.asInt()==1){
    digitalWrite(relaypin4, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin4, 0);
  }
}
BLYNK_WRITE(V38) //relay 5
{
  if(param.asInt()==1){
    digitalWrite(relaypin5, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin5, 0);
  }
}
BLYNK_WRITE(V39) //relay 6
{
  if(param.asInt()==1){
    digitalWrite(relaypin6, 1);
  }
  else if(param.asInt()==0){
    digitalWrite(relaypin6, 0);
  }
}
BLYNK_WRITE(V40) //relay 6
{
  buzzeractive=param.asInt();
  BUZZER_PIN=13;
}
BLYNK_WRITE(V41) //relay 6
{
  bhi=param.asInt();
}
BLYNK_WRITE(V42) //relay 6
{
  blo=param.asInt();
}
BLYNK_WRITE(V43) //relay 6
{
  brp=param.asInt();
  beep(bhi,blo,brp);
  Serial.println("beep recived");
}

void checkConnectionOrRestart() {
  unsigned long startAttemptTime = millis();
  bool wifiConnected = false;
  bool blynkConnected = false;

  WiFi.begin(ssid, pass);

  // Wait up to 60 seconds for Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 60000) {
    delay(500);
    Serial.print(".");
    beep(1, 1, 100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi failed, restarting...");
    ESP.restart();
  }

  Serial.println("Wi-Fi connected");
  beep(1, 1, 200);

  // Reset timer for Blynk connection attempt
  startAttemptTime = millis();
  Blynk.connect();

  // Wait up to 60 seconds for Blynk connection
  while (!Blynk.connected() && millis() - startAttemptTime < 60000) {
    delay(500);
    Serial.print("x");
    beep(1, 1, 100);
  }

  if (!Blynk.connected()) {
    Serial.println("Blynk failed, restarting...");
    ESP.restart();
  }

  Serial.println("Blynk connected");
  beep(2, 2, 100);
}
// Core 0 task function - Now handles analog sensing
void codeForCore0(void *pvParameters) {
  Serial.print("Core 0 task started on core: ");
  Serial.println(xPortGetCoreID());

  while (1) { // Infinite loop
    // Voltage sensing
    int totalAdcValue = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      totalAdcValue += analogRead(ANALOG_IN_PIN);
      delay(10);
    }
    float adc_value_avg = totalAdcValue / (float)NUM_SAMPLES;
    float voltage_adc = 0;
    if (adc_chars != NULL) {
      voltage_adc = esp_adc_cal_raw_to_voltage(adc_value_avg, adc_chars) / 1000.0;
    }
    
    float voltage_in = voltage_adc * (R1 + R2) / R2;
    //float vprocess = (voltage_adc-0.17)

    // ACS712 current measurement
    acsAdvVal = ACS.mA_DC();
    acsTotal -= acsReadings[acsReadIndex];
    acsReadings[acsReadIndex] = acsAdvVal;
    acsTotal += acsReadings[acsReadIndex];
    acsReadIndex = (acsReadIndex + 1) % NUM_SAMPLES;
    acsValAvrg = acsTotal / NUM_SAMPLES;

    // Update shared variables
    shared_voltage_in = voltage_in;
    shared_voltage_adc = voltage_adc;
    shared_acsAdvVal = acsAdvVal;
    shared_acsValAvrg = acsValAvrg;

    // Print results
    Serial.print("Measured Voltage = ");
    Serial.println(shared_voltage_in, 2);
    
    Serial.print("Raw ADC: 2763    = ");
    Serial.println(shared_acsAdvVal);
    Serial.print("Smoothed ma:     = ");
    Serial.println(shared_acsValAvrg);
    //Serial.print(" | Smoothed amp: ");
    //Serial.print((shared_acsValAvrg)/1000);

    vTaskDelay(pdMS_TO_TICKS(core0delay)); // 1 second delay
  }
}

// Core 1 task function
void codeForCore1(void *pvParameters) {
  Serial.print("Core 1 task started on core: ");
  Serial.println(xPortGetCoreID());
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  beep(1,1,200);
  Blynk.config(auth,server,port);  // Only sets up the auth, doesn't manage WiFi
  Blynk.connect();     // Optional: block until connected
  beep(2,2,100);
  ArduinoOTA.setPassword("realme123"); // Choose any password you like
  ArduinoOTA.setHostname("power-core");
  ArduinoOTA.begin();
  beep(3,3,100);
  Serial.println("---------------------------------OTA Ready------------------------------------");
  while (1) { // Infinite loop
    ArduinoOTA.handle();
    int adcValue = analogRead(THERMISTOR_PIN);
    float voltage = adcValue * (3.3 / 4095.0); // Convert ADC to voltage
    float resistance = SERIES_RESISTOR * (3.3 / voltage - 1); // Calculate resistance

    // Steinhart-Hart Equation for temperature conversion
    float temperature = 1.0 / (1.0 / (NOMINAL_TEMPERATURE + 273.15) + 
                    (1.0 / B_COEFFICIENT) * log(resistance / NOMINAL_RESISTANCE)) - 273.15;

    Serial.print("Temperature:     = ");
    Serial.print(temperature);
    temp=temperature;
    Serial.println(" °C");

  // Send Core 0 measurements to Blynk
    Blynk.virtualWrite(V4, temperature);
    Blynk.virtualWrite(V5, shared_voltage_in);
    Blynk.virtualWrite(V6, shared_voltage_adc);
    Blynk.virtualWrite(V7, shared_acsAdvVal);
    Blynk.virtualWrite(V8, shared_acsValAvrg);
    Blynk.virtualWrite(V9, (shared_acsValAvrg-200.0)/1000.0);
    Blynk.virtualWrite(V10, shared_voltage_adc*5.329);
    if (Blynk.connected()) {
    Blynk.run();
    digitalWrite(2,1);
  } else {
    digitalWrite(2,0);
    checkConnectionOrRestart();
    //Blynk.begin(ssid, pass, auth, server, port);
    //WiFi.reconnect();
    //Blynk.connect();
  }
    delay(core1delay);
  }
}

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(relaypin1, OUTPUT);
  pinMode(relaypin2, OUTPUT);
  pinMode(relaypin3, OUTPUT);
  pinMode(relaypin4, OUTPUT);
  pinMode(relaypin5, OUTPUT);
  pinMode(relaypin6, OUTPUT);
  digitalWrite(relaypin1, 0);
  digitalWrite(relaypin2, 0);
  digitalWrite(relaypin3, 0);
  digitalWrite(relaypin4, 0);
  digitalWrite(relaypin5, 0);
  digitalWrite(relaypin6, 0);
  // Create tasks for both cores
  xTaskCreatePinnedToCore(
    codeForCore0,   // Task function
    "Core0_Task",  // Task name
    4096,           // Stack size (bytes)
    NULL,           // Parameters
    1,              // Priority
    &Core0Task,     // Task handle
    0               // Core ID (0)
  );

  xTaskCreatePinnedToCore(
    codeForCore1,   // Task function
    "Core1_Task",  // Task name
    4096,           // Stack size (bytes)
    NULL,           // Parameters
    1,              // Priority
    &Core1Task,     // Task handle
    1               // Core ID (1)
  );

  // Connect to WiFi and Blynk server
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  // Init RS485 UART
  pzemSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17

  // Init PZEM devices
  pzem1.begin(1, pzemSerial);  // ID 1
  pzem1.preTransmission(preTransmission);
  pzem1.postTransmission(postTransmission);

  pzem3.begin(3, pzemSerial);  // ID 3
  pzem3.preTransmission(preTransmission);
  pzem3.postTransmission(postTransmission);
  timer.setInterval(5000L, []() {
    readPZEM(pzem1, 1);
    readPZEM(pzem3, 3);
  });

  //WiFi.begin(ssid, pass);
  //while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    //Serial.print(".");
  //}
  //Serial.println("\nConnected to WiFi");
  //Blynk.config(auth, server, port);
  //Blynk.connect();
  
  analogReadResolution(12); // Set ADC to 12-bit
  analogSetAttenuation(ADC_11db); // Allows ~3.3V input

  // Allocate memory for ADC calibration characteristics with error checking
  adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
  if (adc_chars == NULL) {
    Serial.println("Failed to allocate memory for ADC calibration");
    // Optionally halt execution or implement fallback
    while(1) delay(1000);
  }
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

  // ACS712 setup
  Serial.println(__FILE__);
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);
  ACS.autoMidPoint();
  Serial.println(ACS.getMidPoint());
  beep(100,100,8);
}

void loop() {
  uint8_t result;
  timer.run();
  if(buzzeractive==1){  
      if(powerstatus!=mirror1){
        if(powerstatus==1){
        playAlertCode(1);
        }
        if(powerstatus==0){
        playAlertCode(2);
        }
        mirror1=powerstatus;
      }
    if(temp>overheat){
      playAlertCode(3);
    }
    
  }
  delay(core2delay);
}
// Shared function to read from PZEM
void readPZEM(ModbusMaster &node, int id) {
  uint8_t result = node.readInputRegisters(0x0000, 10);
  if (result == node.ku8MBSuccess) {
    float v  = node.getResponseBuffer(0x00) / 10.0;
    float c  = node.getResponseBuffer(0x01) / 1000.0;
    float p  = node.getResponseBuffer(0x03) / 10.0;
    float e  = node.getResponseBuffer(0x05) / 1000.0;
    float f  = node.getResponseBuffer(0x07) / 10.0;
    float pf = node.getResponseBuffer(0x08) / 100.0;
    float alarm = node.getResponseBuffer(0x09);

    Serial.printf("PZEM-%d: V=%.1fV, I=%.3fA, P=%.1fW, E=%.3fkWh, F=%.1fHz, PF=%.2f, Alarm=%.0f\n",
                  id, v, c, p, e, f, pf, alarm);

    // Send data to different Blynk Virtual Pins
    if (id == 1) {
      Blynk.virtualWrite(V21, v);
      Blynk.virtualWrite(V22, c);
      Blynk.virtualWrite(V23, p);
      Blynk.virtualWrite(V24, e);
      Blynk.virtualWrite(V25, f);
      Blynk.virtualWrite(V26, pf);
      Blynk.virtualWrite(V27, alarm);
      Blynk.virtualWrite(V28, 1); //offlne
      alarm1=alarm;
      powerstatus=1;
      if(v>overv){
      playAlertCode(4);
    }
    if(buzzeractive==1){
      if(v<underv){
        playAlertCode(6);
      }
      if(p>overload){
        playAlertCode(5);
      }
      if(alarm1!=0){
        beep(100,100,8);
      }
    }
      

    } else if (id == 3) {
      Blynk.virtualWrite(V11, v);
      Blynk.virtualWrite(V12, c);
      Blynk.virtualWrite(V13, p);
      Blynk.virtualWrite(V14, e);      
      Blynk.virtualWrite(V15, f);
      Blynk.virtualWrite(V16, pf);
      Blynk.virtualWrite(V17, alarm);
      Blynk.virtualWrite(V18, 1); //offlne
      inverterstatus=1;
      if(buzzeractive==1){
        if(v>overv){
          playAlertCode(4);
          }
        if(v<underv){
          playAlertCode(6);
          }
        if(p>overload){
          playAlertCode(5);
          }
        if(alarm1!=0){
          beep(100,100,8);
        }
      }
    }
  } else {
    Serial.printf("PZEM-%d read failed (Code: %02X)\n", id, result);
     if (id == 1) { 
      Blynk.virtualWrite(V21, 0);
      Blynk.virtualWrite(V22, 0);
      Blynk.virtualWrite(V23, 0);
      Blynk.virtualWrite(V25, 0);
      Blynk.virtualWrite(V26, 0);
      Blynk.virtualWrite(V28, 0);// Offline
      powerstatus=0;
    } else if (id == 3) {
      Blynk.virtualWrite(V11, 0);
      Blynk.virtualWrite(V12, 0);
      Blynk.virtualWrite(V13, 0);
      Blynk.virtualWrite(V15, 0);
      Blynk.virtualWrite(V16, 0);
      Blynk.virtualWrite(V18, 0);// Offline
      inverterstatus=0;
    }
  }
}
