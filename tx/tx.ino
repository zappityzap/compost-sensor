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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    if (!Serial) delay(100);
  }
  delay(1000);

  Serial.println("\n\n\nCompost Temperature TX");

  // LoRa radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.print("LoRa radio initialized.");

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
  Serial.print("Short ID: "); Serial.println(shortID);

  // Battery
  analogReference(AR_DEFAULT);

  // Temperature
  thermo.begin(MAX31865_3WIRE);

  // Soil
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("Seesaw started. Version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  Serial.println("Setup completed.");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  int rawBatteryValue = analogRead(A7);
  float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
  float soilTemperature = thermo.temperature(RNOMINAL, RREF);

  // air temp sensor on moisture sensor is +/- 2 *C
  float airTemperature = ss.getTemp();

  // capacitance max is 1023, needs calibration for each sensor
  uint16_t wetness = ss.touchRead(0);

  static float soilTempReadings[NUM_READINGS];
  static float batteryReadings[NUM_READINGS];
  static float airTempReadings[NUM_READINGS];
  static float wetnessReadings[NUM_READINGS];
  static bool initialized = false;
  static int index = 0;
  static float soilTemperatureAverage = 0;
  static float batteryVoltageAverage = 0;
  static float airTemperatureAverage = 0;
  static float wetnessAverage = 0;

  if (!initialized) {
    for (int i = 0; i < NUM_READINGS; i++) {
        soilTempReadings[i] = soilTemperature;
        batteryReadings[i] = batteryVoltage;
        airTempReadings[i] = airTemperature;
        wetnessReadings[i] = wetness;
    }
    soilTemperatureAverage = soilTemperature;
    batteryVoltageAverage = batteryVoltage;
    airTemperatureAverage = airTemperature;
    wetnessAverage = wetness;
    initialized = true;
  }
  soilTempReadings[index] = soilTemperature;
  batteryReadings[index] = batteryVoltage;
  airTempReadings[index] = airTemperature;
  wetnessReadings[index] = wetness;
  index = (index + 1) % NUM_READINGS;

  soilTemperatureAverage = calculateMovingAverage(soilTempReadings, NUM_READINGS);
  batteryVoltageAverage = calculateMovingAverage(batteryReadings, NUM_READINGS);
  airTemperatureAverage = calculateMovingAverage(airTempReadings, NUM_READINGS);
  wetnessAverage = calculateMovingAverage(wetnessReadings, NUM_READINGS);

  Serial.print("TX: ");
  Serial.print("Soil Temp: "); Serial.print(soilTemperatureAverage, 2);
  Serial.print(", Air Temp: "); Serial.print(airTemperatureAverage, 2);
  Serial.print(", Wetness: "); Serial.print(wetness, 1);
  Serial.print(", Battery: "); Serial.print(batteryVoltageAverage, 2);
  Serial.println();

  // Create the JSON document
  JsonDocument doc;
  // doc["id"] = uniqueID;
  doc["id"] = shortID;
  doc["battery"] = floatToString(batteryVoltageAverage, 2);
  doc["soil_temp"] = floatToString(soilTemperatureAverage, 2);
  doc["air_temp"] = floatToString(airTemperatureAverage, 2);
  doc["wetness"] = floatToString(wetnessAverage, 1);

  // Serialize JSON document
  char jsonBuffer[255];
  serializeJson(doc, jsonBuffer);
  // Serial.print("JSON length: "); Serial.print(strlen(jsonBuffer));
  // Serial.print(" "); Serial.println(jsonBuffer);

  // Send JSON
  rf95.send((uint8_t *)jsonBuffer, strlen(jsonBuffer));
  rf95.waitPacketSent();

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
