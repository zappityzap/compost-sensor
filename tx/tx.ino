#include <RTCZero.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include "Adafruit_seesaw.h"

#include "config.h"

// https://edgecollective.io/notes/proto1/
// https://gist.githubusercontent.com/dwblair/b69a20dcf87314348bac970db574a723/raw/ca1f230cd33d78f76a67ec50a37ac1487adeb9b3/zerosleep.ino

RTCZero zerortc;

const byte alarmSeconds = 0;
const byte alarmMinutes = TRANSMIT_INTERVAL;
const byte alarmHours = 0;

volatile bool alarmFlag = false; // Start awake

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

  // Battery
  analogReference(AR_DEFAULT);

  // Temperature
  Serial.println("TX: Starting MAX31865");
  thermo.begin(MAX31865_3WIRE);
  Serial.println("TX: Looking for MAX31865");
  if (thermo.readRTD() != 0) {
    thermoPresent = true;
    Serial.println("TX: MAX31865 found.");
  } else {
    Serial.println("TX: MAX31865 not found.");
  }

  // Soil
  Serial.println("TX: Looking for Seesaw, may take 30 seconds to fail");
  if (ss.begin(0x36)) {
    seesawPresent = true;
    Serial.print("TX: Seesaw started. Version: ");
    Serial.println(ss.getVersion(), HEX);
  } else {
    Serial.println("TX: Seesaw not found.");
  }

  zerortc.begin(); // Set up clocks and such
  resetAlarm();
  zerortc.attachInterrupt(alarmMatch);

  Serial.println("TX: Setup completed.");
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Pausing for 10 seconds, last chance to flash it...");
  delay(10000);
  Serial.println("Oh no, bro! Here we go!");
}

void loop() {
  if (alarmFlag == true) {
    alarmFlag = false;  // Clear flag
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Alarm went off - I'm awake!");
  }
  Serial.println("TX: Loop begin");

  JsonDocument doc;
  Serial.print("TX ");
  Serial.print("ID: "); Serial.print(shortID);
  // doc["id"] = uniqueID;
  doc["id"] = shortID;

  if (thermoPresent) {
    float soilTemperature = thermo.temperature(RNOMINAL, RREF);
    doc["soil_temp"] = floatToString(soilTemperature, 2);
    Serial.print(", Soil Temp: "); Serial.print(soilTemperature, 2);
  }

  if (seesawPresent) {
    float airTemperature = ss.getTemp();
    doc["air_temp"] = floatToString(airTemperature, 2);
    Serial.print(", Air Temp: "); Serial.print(airTemperature, 2);

    // capacitance max is 1023, needs calibration for each sensor
    int wetness = ss.touchRead(0);
    doc["wetness"] = floatToString(wetness, 1);
    Serial.print(", Wetness: "); Serial.print(wetness, 1);
  }

  int rawBatteryValue = analogRead(A7);
  float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
  doc["battery"] = floatToString(batteryVoltage, 4);
  Serial.print(", Battery: "); Serial.print(batteryVoltage, 4);

  Serial.println();

  // Serialize JSON document
  char jsonBuffer[255];
  serializeJson(doc, jsonBuffer);
  // Serial.print("JSON length: "); Serial.print(strlen(jsonBuffer));
  // Serial.print(" "); Serial.println(jsonBuffer);

  // Send JSON
  rf95.send((uint8_t *)jsonBuffer, strlen(jsonBuffer));
  rf95.waitPacketSent();

  Serial.println("TX: Loop end");

  rf95.sleep();
  
  resetAlarm();  // Reset alarm before returning to sleep
  Serial.println("Alarm set, going to sleep now.");
  Serial.println("Serial port output may be unreliable, check RX side for activity.");
  digitalWrite(LED_BUILTIN, LOW);
  zerortc.standbyMode();    // Sleep until next alarm match
}

void alarmMatch(void)
{
  alarmFlag = true; // Set flag
}

void resetAlarm(void) {
  byte seconds = 0;
  byte minutes = 0;
  byte hours = 0;
  byte day = 1;
  byte month = 1;
  byte year = 1;
  
  zerortc.setTime(hours, minutes, seconds);
  zerortc.setDate(day, month, year);

  zerortc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds);
  zerortc.enableAlarm(zerortc.MATCH_HHMMSS);
}

String floatToString(float value, int decimalPlaces) {
  char buffer[10];
  sprintf(buffer, "%.*f", decimalPlaces, value);
  return String(buffer);
}
