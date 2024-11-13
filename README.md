# Compost sensor v3
Multiple, independent sensors broadcasting over LoRa to a base station. Sensors are automatically discovered and added with unqiue short IDs. Base station uploads data to Home Assistant with MQTT over WiFi. Soil temperature, air temperature, LoRa and WiFi RSSI, and battery levels. Appears in Home Assistant as a single device:

![image](https://github.com/user-attachments/assets/42d3b3e0-2441-4c18-acfd-babad10f984e)
![first-pile](https://github.com/user-attachments/assets/d5145ac7-7f69-44aa-a67c-b24bfb05647d)


# Hardware
* Sensor (TX)
    * [Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz - RadioFruit](https://www.adafruit.com/product/3178)
    * [Simple Spring Antenna - 915MHz](https://www.adafruit.com/product/4269)
    * [Platinum RTD Sensor - PT1000 - 3 Wire 1 meter long](https://www.adafruit.com/product/3984)
    * [Adafruit PT1000 RTD Temperature Sensor Amplifier - MAX31865](https://www.adafruit.com/product/3648)
    * [Adafruit STEMMA Soil Sensor - I2C Capacitive Moisture Sensor - JST PH 2mm](https://www.adafruit.com/product/4026)
        * Note: These seem unreliable, and drain the battery. Testing [Chirp!](https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/) sensors next.
* Base Station (RX)
    * [Adafruit Feather M0 WiFi - ATSAMD21 + ATWINC1500](https://www.adafruit.com/product/3010)
    * [Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz - RadioFruit](https://www.adafruit.com/product/3231)
* LiPo batteries
    * [Lithium Ion Cylindrical Battery - 3.7v 2200mAh](https://www.adafruit.com/product/1781)
    * [Lithium Ion Polymer Battery - 3.7v 500mAh Lithium Ion Polymer Battery - 3.7v 500mAh](https://www.adafruit.com/product/1578)
* Stainless steel tube for temperature probe
    * Aliexpress Seiko Metal Store
    * Length 500mm x 1Pcs, OD5mm X ID4.2mm, feels a little too easy to bend. Easy fit in tube, might allow more epoxy for a stronger bond.
    * Length 500mm x 1Pcs, OD5mm X ID4mm, feels stiffer. Need to trim blob of epoxy of on PT1000D for it to fit in.
    * Another 150-250mm in length would be nice.
    * Be careful buying elsewhere, tolerances on eBay stuff is junk.

# Software
* Latest [Ardunio IDE](http://www.arduino.cc/en/Main/Software)
* Legacy v1.8.X Arduino IDE (for WiFi firmware updates), same page, scroll down
* You might need [FUSE](https://github.com/AppImage/AppImageKit/wiki/FUSE) if the AppImage won't start
* [MQTT Explorer](https://mqtt-explorer.com/) is useful for troubleshooting

## Libraries
All of these except RadioHead can be installed through the library manager in Arduino IDE.
* Arduino SAMD
* Adafruit SAMD Boards
* [Adafruit Seesaw](https://github.com/adafruit/Adafruit_Seesaw)
* [Adafruit MAX31865](https://github.com/adafruit/Adafruit_MAX31865)
* [PubSubClient](https://pubsubclient.knolleary.net/)
* [RTCZero](https://www.arduino.cc/reference/en/libraries/rtczero/)
* [Wifi101](https://www.arduino.cc/reference/en/libraries/wifi101/)
* [RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/)
* [home-assistant-integration]( https://github.com/dawidchyrzynski/arduino-home-assistant)
  * Note: [My PR](https://github.com/zappityzap/arduino-home-assistant/tree/pr-setconfig) is required for dynamic sensor creation

# Set up environment
* Install Arduino IDE
* Open preferences and add this additional board manager URL: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
* Tools > Board > Board Manager...
* Search for and install
  * `Arduino SAMD Boards`
  * `Adafruit SAMD Boards`
* Restart the Arudino IDE
* Open the Serial Monitor from the Tools menu
* Try to connect to a board by selecting it from the dropdown
* TODO is there any output with the default sketch? or the blink sketch? no error is probably good enough
* You might need permission to use the serial port: `sudo usermod -aG dialout user` (remember to logout/in)
* Install these libraries from the Library Manager (Tools > Manage Libraries...)
  * `Adafruit_MAX31865`
  * `Adafruit Seesaw`
  * `PubSubClient`
  * `RTCZero`
  * `Wifi101`
* Install `RadioHead` by [downloading the latest version](https://www.airspayce.com/mikem/arduino/RadioHead/) and extract the RadioHead folder to `~/Arduino/libraries`
* Install `ArduinoJSON`
  * `cd ~/Arduino/libraries`
  * `git clone https://github.com/bblanchon/ArduinoJson.git`
* Install my PR of `home-assistant-intergration`:
  * `cd ~/Arduino/libraries`
  * `git clone https://github.com/zappityzap/arduino-home-assistant.git`
  * `cd ~/Arduino/libraries/arduino-home-assistant`
  * `git checkout pr-setconfig`
* Restart the Arduino IDE

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
1. Optional: Add your soil moisture sensor calibration values to the TX config.h
1. Upload the RX sketch to the base station
1. Upload the TX sketch to the sensor

# Testing
1. Monitor serial output on each sensor
1. Look for Wifi connected
1. Look for Sensor transmitting
1. Look for Base receiving sensor data
1. Check HA to confirm communication
