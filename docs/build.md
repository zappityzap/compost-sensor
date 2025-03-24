# Tools and Materials
Soldering iron with a smaller tip, Hakko
Breadboard, for holding headers while soldering them to the board
Solder, what diameter?
Jumper wires
Phillips screwdrivers (regular and small)

# Parts
* Base Station (RX)
    * [Adafruit Feather M0 WiFi - ATSAMD21 + ATWINC1500](https://www.adafruit.com/product/3010)
    * [Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz - RadioFruit](https://www.adafruit.com/product/3231)
    * [Simple Spring Antenna - 915MHz](https://www.adafruit.com/product/4269)
* Sensor (TX)
    * [Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz - RadioFruit](https://www.adafruit.com/product/3178)
    * [Simple Spring Antenna - 915MHz](https://www.adafruit.com/product/4269)
    * [Platinum RTD Sensor - PT1000 - 3 Wire 1 meter long](https://www.adafruit.com/product/3984)
    * [Adafruit PT1000 RTD Temperature Sensor Amplifier - MAX31865](https://www.adafruit.com/product/3328)
    * [Adafruit STEMMA Soil Sensor - I2C Capacitive Moisture Sensor - JST PH 2mm](https://www.adafruit.com/product/4026)
        * Note: These seem unreliable, and drain the battery. Testing [Chirp!](https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/) sensors next.
    * [Lithium Ion Cylindrical Battery - 3.7v 2200mAh](https://www.adafruit.com/product/1781)
        * It's good to have a spare for quick swap, because charging takes a while.
        * [Adafruit Micro Lipo - USB LiIon/LiPoly charger - v2](https://www.adafruit.com/product/1304) for charging.
    * [Adafruit Flanged Weatherproof Enclosure With PG-7 Cable Glands](https://www.adafruit.com/product/3931)

* Stainless steel tube for temperature probe
    * Aliexpress Seiko Metal Store
    * Length 500mm x 1Pcs, OD5mm X ID4.2mm, feels a little too easy to bend. Easy fit in tube, might allow more epoxy for a stronger bond.
    * Length 500mm x 1Pcs, OD5mm X ID4mm, feels stiffer. Need to trim blob of epoxy of on PT1000D for it to fit in.
    * Another 150-250mm in length would be nice.
    * Be careful buying elsewhere, tolerances on eBay stuff is junk.

# Build

## Base station (RX)
1. Solder headers and connectors onto the boards
1. Solder the spring antenna 

## Sensor (TX)
1. Solder the headers onto the feather
    * Snap the header strip for the shorter side to length
    * Put the headers into the breadboard, long side down
    * Set the feather board right-side up, on top of the headers
        * Both sides have labels but the bottom side is easier to read
    * Solder each pin
1. Solder the spring antenna onto the feather
1. Solder the 2/3 wire pads together on the max31865 board
1. Solder the header strip onto the top side of the max31865 board
    * It's easier to connect the pins if they're on the same side as the labels
    
1. Insert the thermcouple into the stainless steel tube
1. Apply epoxy to secure the thermocouple
    * Leave about half of it sticking out the end of the tube
1. Wait for the epoxy to cure
1. Fold the forked connectors in half so they'll fit into the terminal on the max31865 board
1. Insert both red connecotrs into the terminals on F+ side and secure with the screw
1. Insert the blue connector into either of the terminals on the F- side and secure with the screw
1. Connect the max31865 board to the feather using jumper cables as follows:
    * VIN -> 3.3V
    * GND -> GND
    * CLK -> 13
    * SDO -> 12
    * SDI -> 11
    * CS -> 10
1. Attach the battery to the feather
1. Put all the parts into the weatherproof box

