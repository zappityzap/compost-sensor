#include <SPI.h>
#include <WiFi101.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <ArduinoHA.h>

#include "config.h"

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
HASensorNumber sensorBattery("sensorBattery", HASensorNumber::PrecisionP2);
HASensorNumber soilTemperature("soilTemperature", HASensorNumber::PrecisionP2);
HASensorNumber airTemperature("airTemperature", HASensorNumber::PrecisionP2);
HASensorNumber wetness("wetness", HASensorNumber::PrecisionP1);
HASensorNumber loraRSSI("loraRSSI", HASensorNumber::PrecisionP1);

String uniqueID = "";

#define NUM_READINGS 5

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

  baseBattery.setIcon("mdi:battery");
  baseBattery.setName("Base Battery");
  baseBattery.setUnitOfMeasurement("V");

  loraRSSI.setIcon("mdi:antenna");
  loraRSSI.setName("Sensor LoRa RSSI");
  loraRSSI.setUnitOfMeasurement("dBm");

  soilTemperature.setIcon("mdi:temperature-celsius");
  soilTemperature.setName("Soil Temperature");
  soilTemperature.setUnitOfMeasurement("C");

  sensorBattery.setIcon("mdi:battery");
  sensorBattery.setName("Sensor Battery");
  sensorBattery.setUnitOfMeasurement("V");

  airTemperature.setIcon("mdi:temperature-celsius");
  airTemperature.setName("Air Temperature");
  airTemperature.setUnitOfMeasurement("C");

  wetness.setIcon("mdi:ski-water");
  wetness.setName("Wetness");
  wetness.setUnitOfMeasurement("Wetness");

  // mqtt.setKeepAlive(60); // set the keep alive interval to 60 seconds, default is 15 seconds
  mqtt.begin(MQTT_BROKER_ADDR, MQTT_BROKER_PORT, MQTT_USER, MQTT_PASS);

  Serial.println("Setup completed.");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static float baseBatteryReadings[NUM_READINGS];
  static float loraRSSIReadings[NUM_READINGS];
  static bool initialized = false;
  static int index = 0;
  static float baseBatteryAverage = 0;
  static float loraRSSIAverage = 0;

  mqtt.loop();

  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);

      int loraRSSIValue = rf95.lastRssi();

      // Base station battery voltage
      int rawBatteryValue = analogRead(A7);
      float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
      if (!initialized) {
        for (int i = 0; i < NUM_READINGS; i++) {
            baseBatteryReadings[i] = batteryVoltage;
            loraRSSIReadings[i] = loraRSSIValue;
        }
        baseBatteryAverage = batteryVoltage;
        loraRSSIAverage = loraRSSIValue;
        initialized = true;
      }
      baseBatteryReadings[index] = batteryVoltage;
      loraRSSIReadings[index] = loraRSSIValue;
      index = (index + 1) % NUM_READINGS;
      baseBatteryAverage = calculateMovingAverage(baseBatteryReadings, NUM_READINGS);
      loraRSSIAverage = calculateMovingAverage(loraRSSIReadings, NUM_READINGS);
      baseBattery.setValue(baseBatteryAverage, true);
      loraRSSI.setValue(loraRSSIAverage, true);

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
      
      soilTemperature.setValue(soilTemperatureValue, true);
      sensorBattery.setValue(sensorBatteryValue, true);
      airTemperature.setValue(airTemperatureValue, true);
      wetness.setValue(wetnessValue, true);

      Serial.print("RX ");      
      Serial.print("ID: "); Serial.print(sensorId);
      Serial.print(", Soil Temp: "); Serial.print(soilTemperatureValue, 2);
      Serial.print(", Air Temp: "); Serial.print(airTemperatureValue, 2);
      Serial.print(", Wetness: "); Serial.print(wetnessValue, 1);
      Serial.print(", Battery: "); Serial.print(sensorBatteryValue, 2);
      Serial.print(", LoRa RSSI: "); Serial.print(loraRSSIAverage, 1);
      Serial.println();

      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }
  
  // ADD BASE WIFI RSSI TOO

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
