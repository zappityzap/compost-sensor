#include <SPI.h>
#include <WiFi101.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <ArduinoHA.h>
#include <map>

#include "config.h"

#define NUM_READINGS 5

#define RFM95_RST 11  // "A"
#define RFM95_CS  10  // "B"
#define RFM95_INT  6  // "D"

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

int wifiStatus = WL_IDLE_STATUS;

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 40);
HASensorNumber baseBattery("baseBattery", HASensorNumber::PrecisionP2);

std::map<String, float[NUM_READINGS]> loraRssiReadingsMap;
std::map<String, int> loraRssiIndexMap;
std::map<String, float> loraRssiAverageMap;

std::map<String, HASensorNumber*> loraRssiMap;
std::map<String, String*> loraRssiEntityMap;
std::map<String, String*> loraRssiNameMap;

std::map<String, HASensorNumber*> sensorBatteryMap;
std::map<String, String*> sensorBatteryEntityMap;
std::map<String, String*> sensorBatteryNameMap;

std::map<String, HASensorNumber*> soilTemperatureMap;
std::map<String, String*> soilTemperatureEntityMap;
std::map<String, String*> soilTemperatureNameMap;

std::map<String, HASensorNumber*> airTemperatureMap;
std::map<String, String*> airTemperatureEntityMap;
std::map<String, String*> airTemperatureNameMap;

std::map<String, HASensorNumber*> wetnessMap;
std::map<String, String*> wetnessEntityMap;
std::map<String, String*> wetnessNameMap;

String uniqueID = "";

void setup() {
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup Serial port
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    if (!Serial) delay(100);
  }
  delay(100);
  Serial.println("\n\n\nCompost Temperature RX");

  // Unique ID
  uint32_t id[4];
  uint32_t *idAddress = (uint32_t *)0x0080A00C;
  for (int i = 0; i < 4; i++) {
    id[i] = idAddress[i];
    uniqueID += String(id[i], HEX);
    if (i < 3) uniqueID += "-";
  }
  device.setUniqueId((byte*)uniqueID.c_str(), uniqueID.length());  
  Serial.print("Unique ID: "); Serial.println(uniqueID);

  // Setup WiFi
  WiFi.setPins(8, 7, 4, 2);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(1000);
  }
  printWiFiStatus();

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
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.print("LoRa radio initialized.");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(" setFrequency failed");
    while (1);
  }
  Serial.print(" Frequency: "); Serial.print(RF95_FREQ); Serial.println(" MHz");

  rf95.setTxPower(20, false);

  // Setup Home Assistant MQTT
  device.setName("Compost RX");
  device.setSoftwareVersion("1.0.0");
  device.enableSharedAvailability();
  device.enableLastWill();
  device.enableExtendedUniqueIds(); 

  baseBattery.setIcon("mdi:battery");
  baseBattery.setName("Base Battery");
  baseBattery.setUnitOfMeasurement("V");

  // mqtt.setKeepAlive(60); // set the keep alive interval to 60 seconds, default is 15 seconds
  mqtt.begin(MQTT_BROKER_ADDR, MQTT_BROKER_PORT, MQTT_USER, MQTT_PASS);

  Serial.println("Setup completed.");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static float baseBatteryReadings[NUM_READINGS];
  static float loraRssiReadings[NUM_READINGS];
  static bool initialized = false;
  static int index = 0;
  static float baseBatteryAverage = 0;
  static float loraRssiAverage = 0;
  bool published = false;

  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println();

      int loraRssiValue = rf95.lastRssi();

      // Base station battery voltage
      int rawBatteryValue = analogRead(A7);
      float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
      if (!initialized) {
        for (int i = 0; i < NUM_READINGS; i++) {
            baseBatteryReadings[i] = batteryVoltage;
        }
        baseBatteryAverage = batteryVoltage;
        initialized = true;
      }
      baseBatteryReadings[index] = batteryVoltage;
      index = (index + 1) % NUM_READINGS;
      baseBatteryAverage = calculateMovingAverage(baseBatteryReadings, NUM_READINGS);
      published = baseBattery.setValue(baseBatteryAverage);
      Serial.println(published ? "Published baseBatteryAverage" : "Failed to publish baseBatteryAverage");

      buf[len] = '\0';
      Serial.print("JSON length: "); Serial.print(strlen((char *)buf));
      Serial.print(" "); Serial.println((char *)buf);

      JsonDocument doc;
      deserializeJson(doc, (char*)buf);
      
      String sensorId = doc["id"];
      float soilTemperatureValue = doc["soil_temp"];
      float sensorBatteryValue = doc["battery"];
      float airTemperatureValue = doc["air_temp"];
      float wetnessValue = doc["wetness"];
      
      // Dynamically create sensors if they don't already exist
      if (loraRssiMap.find(sensorId) == loraRssiMap.end()) {
        loraRssiEntityMap[sensorId] = new String("S" + sensorId + "_loraRssi");
        loraRssiNameMap[sensorId] = new String("S" + sensorId + " LoRa RSSI");
        loraRssiMap[sensorId] = new HASensorNumber(loraRssiEntityMap[sensorId]->c_str(), HASensorNumber::PrecisionP1);
        loraRssiMap[sensorId]->setIcon("mdi:antenna");
        loraRssiMap[sensorId]->setName(loraRssiNameMap[sensorId]->c_str());
        loraRssiMap[sensorId]->setUnitOfMeasurement("dBm");

        // Initialize RSSI readings and index
        for (int i = 0; i < NUM_READINGS; i++) {
            loraRssiReadingsMap[sensorId][i] = 0;
        }
        loraRssiIndexMap[sensorId] = 0;
        loraRssiAverageMap[sensorId] = 0;
      }
      if (soilTemperatureMap.find(sensorId) == soilTemperatureMap.end()) {
        soilTemperatureEntityMap[sensorId] = new String("S" + sensorId + "_soilTemp");
        soilTemperatureNameMap[sensorId] = new String("S" + sensorId + " Soil Temperature");
        soilTemperatureMap[sensorId] = new HASensorNumber(soilTemperatureEntityMap[sensorId]->c_str(), HASensorNumber::PrecisionP2);
        soilTemperatureMap[sensorId]->setIcon("mdi:temperature-celsius");
        soilTemperatureMap[sensorId]->setName(soilTemperatureNameMap[sensorId]->c_str());
        soilTemperatureMap[sensorId]->setUnitOfMeasurement("C");
      }
      if (sensorBatteryMap.find(sensorId) == sensorBatteryMap.end()) {
        sensorBatteryEntityMap[sensorId] = new String("S" + sensorId + "_sensorBattery");
        sensorBatteryNameMap[sensorId] = new String("S" + sensorId + " Battery");
        sensorBatteryMap[sensorId] = new HASensorNumber(sensorBatteryEntityMap[sensorId]->c_str(), HASensorNumber::PrecisionP2);
        sensorBatteryMap[sensorId]->setIcon("mdi:battery");
        sensorBatteryMap[sensorId]->setName(sensorBatteryNameMap[sensorId]->c_str());
        sensorBatteryMap[sensorId]->setUnitOfMeasurement("V");
      }
      if (airTemperatureMap.find(sensorId) == airTemperatureMap.end()) {
        airTemperatureEntityMap[sensorId] = new String("S" + sensorId + "_airTemp");
        airTemperatureNameMap[sensorId] = new String("S" + sensorId + " Air Temperature");
        airTemperatureMap[sensorId] = new HASensorNumber(airTemperatureEntityMap[sensorId]->c_str(), HASensorNumber::PrecisionP2);
        airTemperatureMap[sensorId]->setIcon("mdi:temperature-celsius");
        airTemperatureMap[sensorId]->setName(airTemperatureNameMap[sensorId]->c_str());
        airTemperatureMap[sensorId]->setUnitOfMeasurement("C");
      }
      if (wetnessMap.find(sensorId) == wetnessMap.end()) {
        wetnessEntityMap[sensorId] = new String("S" + sensorId + "_wetness");
        wetnessNameMap[sensorId] = new String("S" + sensorId + " Wetness");
        wetnessMap[sensorId] = new HASensorNumber(wetnessEntityMap[sensorId]->c_str(), HASensorNumber::PrecisionP1);
        wetnessMap[sensorId]->setIcon("mdi:ski-water");
        wetnessMap[sensorId]->setName(wetnessNameMap[sensorId]->c_str());
        wetnessMap[sensorId]->setUnitOfMeasurement("Wetness");
      }

      // Update RSSI readings
      int loraRssiIndex = loraRssiIndexMap[sensorId];
      loraRssiReadingsMap[sensorId][index] = loraRssiValue;
      index = (index + 1) % NUM_READINGS;
      loraRssiIndexMap[sensorId] = index;

      // Calculate moving average
      float loraRssiAverage = calculateMovingAverage(loraRssiReadingsMap[sensorId], NUM_READINGS);
      loraRssiAverageMap[sensorId] = loraRssiAverage;

      // Send to Home Assistant
      published = loraRssiMap[sensorId]->setValue(loraRssiAverage);
      Serial.print("loraRssi "); Serial.print(loraRssiMap[sensorId]->uniqueId());
      Serial.println(published ? " Published" : " Failed to publish");

      published = soilTemperatureMap[sensorId]->setValue(soilTemperatureValue);
      Serial.print("soilTemperatureValue "); Serial.print(soilTemperatureMap[sensorId]->uniqueId());
      Serial.println(published ? " Published" : " Failed to publish");

      published = sensorBatteryMap[sensorId]->setValue(sensorBatteryValue);
      Serial.print("sensorBatteryValue "); Serial.print(sensorBatteryMap[sensorId]->uniqueId());
      Serial.println(published ? " Published" : " Failed to publish");

      published = airTemperatureMap[sensorId]->setValue(airTemperatureValue);
      Serial.print("airTemperatureValue "); Serial.print(airTemperatureMap[sensorId]->uniqueId());
      Serial.println(published ? " Published" : " Failed to publish");

      published = wetnessMap[sensorId]->setValue(wetnessValue);
      Serial.print("wetnessValue "); Serial.print(wetnessMap[sensorId]->uniqueId());
      Serial.println(published ? " Published" : " Failed to publish");

      Serial.print("RX ");
      Serial.print("ID: "); Serial.print(sensorId);
      Serial.print(", Soil Temp: "); Serial.print(soilTemperatureValue, 2);
      Serial.print(", Air Temp: "); Serial.print(airTemperatureValue, 2);
      Serial.print(", Wetness: "); Serial.print(wetnessValue, 1);
      Serial.print(", Battery: "); Serial.print(sensorBatteryValue, 2);
      Serial.print(", LoRa RSSI: "); Serial.print(loraRssiAverage, 1);
      Serial.println();

      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }
  
  // ADD BASE WIFI RSSI TOO

  mqtt.loop();

  delay(LISTEN_INTERVAL);
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.print(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print(", IP Address: ");
  Serial.print(ip);

  long rssi = WiFi.RSSI();
  Serial.print(", RSSI: ");
  Serial.println(rssi);
}

float calculateMovingAverage(float readings[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += readings[i];
  }
  return sum / size;
}
