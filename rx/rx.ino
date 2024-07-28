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

HADevice device;
WiFiClient client;
HAMqtt mqtt(client, device);
HASensorNumber baseBattery("baseBattery", HASensorNumber::PrecisionP3);
HASensorNumber sensorBattery("sensorBattery", HASensorNumber::PrecisionP3);
HASensorNumber soilTemperature("soilTemperature", HASensorNumber::PrecisionP3);
HASensorNumber airTemperature("airTemperature", HASensorNumber::PrecisionP3);
HASensorNumber soilCapacitance("soilCapacitance");
HASensorNumber soilMoisture("soilMoisture", HASensorNumber::PrecisionP2);

String uniqueID = "";

#define NUM_READINGS 5

void setup() {
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);

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

  soilTemperature.setIcon("mdi:temperature-celsius");
  soilTemperature.setName("Soil Temperature");
  soilTemperature.setUnitOfMeasurement("C");

  sensorBattery.setIcon("mdi:battery");
  sensorBattery.setName("Sensor Battery");
  sensorBattery.setUnitOfMeasurement("V");

  airTemperature.setIcon("mdi:temperature-celsius");
  airTemperature.setName("Air Temperature");
  airTemperature.setUnitOfMeasurement("C");

  soilCapacitance.setIcon("mdi:ski-water");
  soilCapacitance.setName("Soil Capacitance");
  soilCapacitance.setUnitOfMeasurement("/1023");

  soilMoisture.setIcon("mdi:water");
  soilMoisture.setName("Soil Moisture");
  soilMoisture.setUnitOfMeasurement("%");

  baseBattery.setIcon("mdi:battery");
  baseBattery.setName("Base Battery");
  baseBattery.setUnitOfMeasurement("V");

  // mqtt.setKeepAlive(60); // set the keep alive interval to 60 seconds, default is 15 seconds
  mqtt.begin(MQTT_BROKER_ADDR, MQTT_BROKER_PORT, MQTT_USER, MQTT_PASS);

  Serial.println("Setup completed.");
}

void loop() {
  static float batteryReadings[NUM_READINGS];
  static bool initialized = false;
  static int index = 0;
  static float v_ma = 0;

  mqtt.loop();

  // Base station battery voltage
  int rawBatteryValue = analogRead(A7);
  float batteryVoltage = rawBatteryValue * (3.3 / 1023.0) * 2;
  if (!initialized) {
    for (int i = 0; i < NUM_READINGS; i++) {
        batteryReadings[i] = batteryVoltage;
    }
    v_ma = batteryVoltage;
    initialized = true;
  }
  batteryReadings[index] = batteryVoltage;
  index = (index + 1) % NUM_READINGS;
  v_ma = calculateMovingAverage(batteryReadings, NUM_READINGS);
  baseBattery.setValue(v_ma);

  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("RX ");
      digitalWrite(LED_BUILTIN, HIGH);

      buf[len] = '\0';
      // Serial.println((char*)buf);
      JsonDocument doc;
      deserializeJson(doc, (char*)buf);
      
      String sensorId = doc["id"];
      float soilTemperatureValue = doc["soil_temp"];
      float sensorBatteryValue = doc["battery"];
      float airTemperatureValue = doc["air_temp"];
      int soilCapacitanceValue = doc["soil_cap"];
      int soilMoistureValue = doc["moisture"];
      int rssi = rf95.lastRssi();

      soilTemperature.setValue(soilTemperatureValue);
      sensorBattery.setValue(sensorBatteryValue);
      airTemperature.setValue(airTemperatureValue);
      soilCapacitance.setValue(soilCapacitanceValue);
      soilMoisture.setValue(soilMoistureValue);

      Serial.print("ID: "); Serial.print(sensorId);
      Serial.print(", Soil Temp: "); Serial.print(soilTemperatureValue, 3);
      Serial.print(", Air Temp: "); Serial.print(airTemperatureValue, 3); Serial.print(" C");
      Serial.print(", Soil Capacitance: "); Serial.print(soilCapacitanceValue);
      Serial.print(", Soil Moisture: "); Serial.print(soilMoistureValue); Serial.print("%");
      Serial.print(", Battery: "); Serial.print(sensorBatteryValue, 3);
      Serial.print(", RSSI: "); Serial.print(rssi);
      Serial.println();

      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }
  
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
