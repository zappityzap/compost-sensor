#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include "Adafruit_seesaw.h"

#include "config.h"

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

#define RREF      4300.0
#define RNOMINAL  1000.0

#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_seesaw ss;

String uniqueID = "";
String shortID = "";

#define NUM_READINGS 5

bool thermoPresent = false;
bool seesawPresent = false;

void setup() {
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup Serial port
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    if (!Serial) delay(100);
  }
  delay(1000);

  Serial.println("\n\n\nCompost Temperature TX");

  // Unique ID
  uint32_t id[4];
  uint32_t *idAddress = (uint32_t *)0x0080A00C;
  for (int i = 0; i < 4; i++) {
    id[i] = idAddress[i];
    uniqueID += String(id[i], HEX);
    if (i < 3) uniqueID += "-";
  }
  Serial.print("TX: Unique ID: "); Serial.println(uniqueID);

  // Friendly ID, hopefully unique
  shortID = uniqueID.substring(uniqueID.length() - 3);
  shortID.toUpperCase();
  Serial.print("TX: Short ID: "); Serial.println(shortID);

  // LoRa radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("TX: LoRa radio init failed");
    Serial.println("TX: Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.print("TX: LoRa radio initialized.");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(" setFrequency failed");
    while (1);
  }
  Serial.print(" Frequency: "); Serial.print(RF95_FREQ); Serial.println(" MHz");

  rf95.setTxPower(20, false);

  // Unique ID
  uint32_t id[4];
  uint32_t *idAddress = (uint32_t *)0x0080A00C;
  for (int i = 0; i < 4; i++) {
    id[i] = idAddress[i];
    uniqueID += String(id[i], HEX);
    if (i < 3) uniqueID += "-";
  }
  Serial.print("Unique ID: "); Serial.println(uniqueID);

  // Friendly ID, hopefully unique
  shortID = uniqueID.substring(uniqueID.length() - 3);
  shortID.toUpperCase();
  Serial.print("Short ID: "); Serial.println(shortID);

  // Battery
  analogReference(AR_DEFAULT);

  // Temperature
  thermo.begin(MAX31865_3WIRE);
  if (thermo.readRTD() != 0) {
    thermoPresent = true;
    Serial.println("TX: MAX31865 found.");
  } else {
    Serial.println("TX: MAX31865 not found.");
  }

  // Soil
  if (ss.begin(0x36)) {
    seesawPresent = true;
    Serial.print("TX: Seesaw started. Version: ");
    Serial.println(ss.getVersion(), HEX);
  } else {
    Serial.println("TX: Seesaw not found.");
  }

  Serial.println("TX: Setup completed.");
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Pausing for 10 seconds, last chance to flash it...");
  delay(10000);
  Serial.println("Oh no, bro! Here we go!");
}

void loop() {
  static float soilTempReadings[NUM_READINGS];
  static float airTempReadings[NUM_READINGS];
  static float wetnessReadings[NUM_READINGS];
  static float batteryReadings[NUM_READINGS];

  static bool soilTempInitialized = false;
  static bool airTempInitialized = false;
  static bool wetnessInitialized = false;
  static bool batteryInitialized = false;

  static int index = 0;

  static float soilTemperatureAverage = 0;
  static float airTemperatureAverage = 0;
  static float wetnessAverage = 0;
  static float batteryVoltageAverage = 0;

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("TX: Loop begin");

  JsonDocument doc;
  Serial.print("TX ");
  Serial.print("ID: "); Serial.print(shortID);
  // doc["id"] = uniqueID;
  doc["id"] = shortID;

  if (thermoPresent) {
    float soilTemperature = thermo.temperature(RNOMINAL, RREF);
    if (!soilTempInitialized) {
      for (int i = 0; i < NUM_READINGS; i++) {
        soilTempReadings[i] = soilTemperature;
      }
      soilTemperatureAverage = soilTemperature;
      soilTempInitialized = true;
    }
    soilTempReadings[index] = soilTemperature;
    soilTemperatureAverage = calculateMovingAverage(soilTempReadings, NUM_READINGS);
    doc["soil_temp"] = floatToString(soilTemperatureAverage, 2);
    Serial.print(", Soil Temp: "); Serial.print(soilTemperatureAverage, 2);
  }

  if (seesawPresent) {
    float airTemperature = ss.getTemp();
    if (!airTempInitialized) {
      for (int i = 0; i < NUM_READINGS; i++) {
        airTempReadings[i] = airTemperature;
      }
      airTemperatureAverage = airTemperature;
      airTempInitialized = true;
    }
    airTempReadings[index] = airTemperature;
    airTemperatureAverage = calculateMovingAverage(airTempReadings, NUM_READINGS);
    doc["air_temp"] = floatToString(airTemperatureAverage, 2);
    Serial.print(", Air Temp: "); Serial.print(airTemperatureAverage, 2);

    // capacitance max is 1023, needs calibration for each sensor
    int wetness = ss.touchRead(0);
    if (!wetnessInitialized) {
      for (int i = 0; i < NUM_READINGS; i++) {
        wetnessReadings[i] = wetness;
      }
      wetnessAverage = wetness;
      wetnessInitialized = true;
    }
    wetnessReadings[index] = wetness;
    wetnessAverage = calculateMovingAverage(wetnessReadings, NUM_READINGS);
    doc["wetness"] = floatToString(wetnessAverage, 1);
    Serial.print(", Wetness: "); Serial.print(wetnessAverage, 1);
  }

  int rawBatteryValue = analogRead(A7);
  float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
  if (!batteryInitialized) {
    for (int i = 0; i < NUM_READINGS; i++) {
      batteryReadings[i] = batteryVoltage;
    }
    batteryVoltageAverage = batteryVoltage;
    batteryInitialized = true;
  }
  batteryReadings[index] = batteryVoltage;
  batteryVoltageAverage = calculateMovingAverage(batteryReadings, NUM_READINGS);
  doc["battery"] = floatToString(batteryVoltageAverage, 2);
  Serial.print(", Battery: "); Serial.print(batteryVoltageAverage, 2);

  Serial.println();
  index = (index + 1) % NUM_READINGS;

  // Serialize JSON document
  char jsonBuffer[255];
  serializeJson(doc, jsonBuffer);
  // Serial.print("JSON length: "); Serial.print(strlen(jsonBuffer));
  // Serial.print(" "); Serial.println(jsonBuffer);

  // Send JSON
  rf95.send((uint8_t *)jsonBuffer, strlen(jsonBuffer));
  rf95.waitPacketSent();

  Serial.println("TX: Loop end");
  digitalWrite(LED_BUILTIN, LOW);

  delay(TRANSMIT_INTERVAL);
}

float calculateMovingAverage(float readings[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += readings[i];
  }
  return sum / size;
}

String floatToString(float value, int decimalPlaces) {
  char buffer[10];
  sprintf(buffer, "%.*f", decimalPlaces, value);
  return String(buffer);
}
