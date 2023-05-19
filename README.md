# ESP32IoT-Iceberg
1. Install ESP32 by Espressif Systems Version "2.0.5-2.0.9".
- on File>Preferences>Additional Boards Manager URLs, add "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"  
Caution!! Other versions may have different EEPROMClass methods!

- Board > Boards Manager, search esp32 and install v2.0.5-2.0.9

2. In Tools tab,
- Board > esp32, choose "ESP32 Dev Module"
- Partition Scheme, choose "HugeApp"
- 
## Library dependencies

- ArduinoJson (by Benoit Blanchon) v6.19.4 

- MQTT (by Joel Gaehwiler) v2.5.1

- Sensor Libraries : Adafruit DPS310(Adafruit, v1.1.1), Adafruit SHT4x(Adafruit, v1.0.2), FastIMU(LiquidCGS, v1.0.6)

---------------------
- Socket.io (깃헙 검색 후 설치)

https://github.com/Links2004/arduinoWebSockets

다운로드 후 PC>documents>Arduino>libraries 안에 압축해제

*폴더안에 폴더 들어가지 않도록 주의

Ex) example.zip 해제 -> example 폴더 생기면서 그 안에 또 다시 example폴더가 들어가있는 형태로 압축해제 될 수 있음.



## Development tools

- (android) playstore -> BLE scanner

- (ios) something like BLE scanner

## ToDo:

1. Inhibit low-performance mode if BLE is connected.

2. Sensor integration

sensor-update modules, motion interrupt by imu sensor

3. Button events
