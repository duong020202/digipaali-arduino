# Digipaali Wemos board

Digipaali (2018-2020) is a public-funded project conducted by Häme University of Applied Sciences, and Natural Resources Institute Finland, in cooperation with Finnish innovative farms. The project aims to improve the efficiency of silage bale life-cycle management. This repository includes Arduino code running on a Wemos Lolin D1 Mini Pro v2 board that collects ambient, agricultural data, and communicates with RFID eNur 05WL2 to acquire tag IDs

These instructions will get you a copy of the project up and running on your development board for programming and testing purposes. 

## Prerequisites
### Hardware (could be replaced with compatible components)
* Wemos Lolin D1 Mini Pro v2 (or any board with Wifi module, and serial interfaces: UART, I2C)
* Amphenol Telaire T9602
* Qwiic GPS module
* eNur 05WL2
* Adafruit_ADS1115

### Software

* C/C++ language compiler

## Installing

Clone the repository or download zip file and extract it, in your desire folder:

```
git clone https://github.com/tqbao98/digipaali-arduino.git
```

In ```final.cpp```, change ```ssid```, ```password```, and ```mqtt_server``` to your own configuration

## Running

### PlatformIO

In ```platformio.ini```, edit ```platform```, ```board```, and ```framework``` to match your hardware. Hit **Build**, then **Upload** to run.

### Arduino IDE

* Remove **#include <Arduino.h>** in ```final.cpp```
* Change file extension from ```.cpp``` to ```.ino```
* Install potentially required library via Library Manager

## Behaviour

* The program connects to configured Wifi and MQTT server.
* It continuously reads RFID tags via the reader module, and published found tags to topic ```nurapisample/epc```
* It sends GPS data every 10s to topic ```path```
* When receiving message from topic ```sensors```, it collects temperature, humidity, GPS (and possiblly dry matter) value, and published them to topic ```todatabase```

## Acknowledgments

This repository uses the code from following authors, and libraries (please let me know if your credit is missing):

* [Telaire T9602](https://github.com/AmphenolAdvancedSensors/Telaire/tree/T9602-ChipCap2)
* [Qwiic GPS and TinyGPS](https://github.com/sparkfun/SparkFun_GPS_Breakout_XA1110_Qwiic)
* [NordicID](https://github.com/NordicID/nur_nurapi_micro/tree/master/arduino)
* [Adafruit ADS1x15](https://github.com/adafruit/Adafruit_ADS1X15)
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
* [PubSubClient](https://github.com/knolleary/pubsubclient)
* [ESP8266WiFi](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi)

## Contributors

* [Khoa Dang](https://github.com/DankMinhKhoa)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.md) file for details