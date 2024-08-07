# Compost sensor v3
Multiple, independent sensors broadcasting over LoRa to a base station. Sensors are automatically discovered and added with unqiue short IDs. Base station uploads data to Home Assistant with MQTT over WiFi. Soil temperature, air temperature, LoRa and WiFi RSSI, and battery levels. Appears in Home Assistant as a single device:

![image](https://github.com/user-attachments/assets/2a2510b2-a9e9-4e8b-8585-05f4cf96f634)

# Hardware
* Sensor (TX)
    * [Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz - RadioFruit](https://www.adafruit.com/product/3178)
    * [Simple Spring Antenna - 915MHz](https://www.adafruit.com/product/4269)
    * [Platinum RTD Sensor - PT1000 - 3 Wire 1 meter long](https://www.adafruit.com/product/3984)
    * [Adafruit PT1000 RTD Temperature Sensor Amplifier - MAX31865](https://www.adafruit.com/product/3328)
    * [Adafruit STEMMA Soil Sensor - I2C Capacitive Moisture Sensor - JST PH 2mm](https://www.adafruit.com/product/4026)
* Base Station (RX)
    * [Adafruit Feather M0 WiFi - ATSAMD21 + ATWINC1500](https://www.adafruit.com/product/3010)
    * [Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz - RadioFruit](https://www.adafruit.com/product/3231)
* LiPo batteries
    * [Lithium Ion Cylindrical Battery - 3.7v 2200mAh](https://www.adafruit.com/product/1781)
    * [Lithium Ion Polymer Battery - 3.7v 500mAh Lithium Ion Polymer Battery - 3.7v 500mAh](https://www.adafruit.com/product/1578)

# Software
* Latest [Ardunio IDE](http://www.arduino.cc/en/Main/Software)
* Legacy v1.8.X Arduino IDE (for WiFi firmware updates)

## Libraries
All of these except RadioHead can be installed through the library manager in Arduino IDE.
* Arduino SAMD
* Adafruit SAMD Boards
* [RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/) (get the latest version instead of Adafruit github copy)
* [Wifi101](https://www.arduino.cc/reference/en/libraries/wifi101/)
* [home-assistant-integration]( https://github.com/dawidchyrzynski/arduino-home-assistant)
  * Note: [My PR](https://github.com/zappityzap/arduino-home-assistant/tree/pr-setconfig) is required for dynamic sensor creation
* [Adafruit Seesaw](https://github.com/adafruit/Adafruit_Seesaw)

# Set up environment
* Add board to Arduino IDE
* Install libraries

# Update Firmware
1. Open Arduino IDE (latest)
1. Open Wifi101 Firmware Updater sketch
1. Add this line to setup(): ```WiFi.setPins(8,7,4,2);```
1. Upload Wifi101 firmware updater sketch example
1. Close Arduino IDE and open legacy Arduino IDE
1. Open Wifi101 Firmware updater from Tools menu
1. Select correct port
1. Test connection
1. Update firmware

# Home Assistant
1. Set up an MQTT broker like Mosquito
1. Create a new HA user dedicated to the sensor

# Configure
Both RX and TX sketches need a config.h
1. Copy config.h.example to config.h
1. Add your WiFi and MQTT details to the RX config.h
1. Add your soil moisture sensor calibration values to the TX config.h
1. Upload the RX sketch to the base station
1. Upload the TX sketch to the sensor

# Testing
1. Monitor serial output on each sensor
1. Look for Wifi connected
1. Look for Sensor transmitting
1. Look for Base receiving sensor data
1. Check HA to confirm communication
