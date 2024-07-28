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

#define NUM_READINGS 5

void setup() {
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
  float temperature = thermo.temperature(RNOMINAL, RREF);
  
  static float tempReadings[NUM_READINGS];
  static float batteryReadings[NUM_READINGS];
  static bool initialized = false;
  static int index = 0;
  static float t_ma = 0;
  static float v_ma = 0;

  if (!initialized) {
    for (int i = 0; i < NUM_READINGS; i++) {
        tempReadings[i] = temperature;
        batteryReadings[i] = batteryVoltage;
    }
    t_ma = temperature;
    v_ma = batteryVoltage;
    initialized = true;
  }
  tempReadings[index] = temperature;
  batteryReadings[index] = batteryVoltage;
  index = (index + 1) % NUM_READINGS;

  t_ma = calculateMovingAverage(tempReadings, NUM_READINGS);
  v_ma = calculateMovingAverage(batteryReadings, NUM_READINGS);

  // Soil percentage mapping
  // https://forums.adafruit.com/viewtopic.php?t=211625&hilit=soil

  // temp is +/- 2C
  float ss_temp = ss.getTemp();
  // capacitance max is 1023, needs calibration for each sensor
  uint16_t ss_cap = ss.touchRead(0);

  // Map the capacitive value to a percentage
  float moisturePercentage = (ss_cap - DRY_VALUE) * 100 / (WET_VALUE - DRY_VALUE);
  if (moisturePercentage < 0) {
      moisturePercentage = 0;
  } else if (moisturePercentage > 100) {
      moisturePercentage = 100;
  }

  Serial.print("TX: ");
  Serial.print("Soil Temp: "); Serial.print(t_ma, 3);
  Serial.print(", Air Temp: "); Serial.print(ss_temp, 3);
  Serial.print(", Soil Capacitance: "); Serial.print(ss_cap);
  Serial.print(", Soil Moisture: "); Serial.print(moisturePercentage, 0); Serial.print("%");
  Serial.print(", Battery: "); Serial.print(v_ma, 3);
  Serial.println();

  // Create the JSON document
  JsonDocument doc;
  doc["id"] = uniqueID;
  doc["battery"] = floatToString(v_ma, 3);
  doc["soil_temp"] = floatToString(t_ma, 3);
  doc["air_temp"] = floatToString(ss_temp, 3);
  doc["soil_cap"] = ss_cap;
  doc["moisture"] = moisturePercentage;

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
