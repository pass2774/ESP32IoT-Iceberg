/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
// TEST
   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "eepromCustom.h"
#include "wifi_custom.h"
#include "SerialUI.h"
#include "spiffsCustom.h"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}



/***********************************************************************
  Adafruit MQTT Library ESP32 Adafruit IO SSL/TLS example

  Use the latest version of the ESP32 Arduino Core:
    https://github.com/espressif/arduino-esp32

  Works great with Adafruit Huzzah32 Feather and Breakout Board:
    https://www.adafruit.com/product/3405
    https://www.adafruit.com/products/4172

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  Modified by Brent Rubell for Adafruit Industries
  MIT license, all text above must be included in any redistribution
 **********************************************************************/
#include <WiFi.h>
#include <MQTT.h>

// const char ssid[] = "JoonhwaHotSpot";
// const char pass[] = "iepstt2774";

const char ssid[] = "ANTS Place";
const char pass[] = "ants@1681";

// WiFiClient net;
MQTTClient MQTTclient;

unsigned long lastMillis = 0;

#define LEN_ACCSMPL 30
#define LEN_PRSSMPL 20
#define LEN_TRHSMPL 20
int8_t acc[LEN_ACCSMPL+2][3];
uint32_t prs[LEN_PRSSMPL+6];
int16_t trh[LEN_TRHSMPL+6];

/************************* WiFi Access Point *********************************/
/// MQTT

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!MQTTclient.connect("arduino", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  MQTTclient.subscribe("perpet/SerialNumber/acc");
  MQTTclient.subscribe("perpet/SerialNumber/prs");
  MQTTclient.subscribe("perpet/SerialNumber/trh");
  MQTTclient.subscribe("perpet/SerialNumber/CMD");
  // client.unsubscribe("/hello");
}


// void messageReceivedAdvanced(MQTTClient *client, char topic[], char bytes[], int length) {
  // String topic2 = topic;
  // String payload = bytes;
  // Serial.println("topic:"+topic2);
  // Serial.print("len:");
  // Serial.println(length);

void messageReceived(String &topic, String &payload) {
  // Serial.println("incoming: " + topic + " - " + payload);
  if(topic == "perpet/SerialNumber/acc"){
    // const char* rxPacket=payload.c_str();
    char rxPacket[LEN_ACCSMPL+2];
    payload.toCharArray(rxPacket,(LEN_ACCSMPL+2),0);
    Serial.println("******");      
    Serial.println("decoded:");
    for(int i=0;i<(LEN_ACCSMPL+2);i++){
      for(int j=0;j<3;j++){
        Serial.print((int8_t)rxPacket[i*3+j]);
        Serial.print(",");
      }
      Serial.println();
    }
    Serial.println("******");      
  }
  if(topic == "perpet/SerialNumber/trh"){
    const char* rxPacket=payload.c_str();
    Serial.println("******");      
    Serial.println("decoded:");
    for(int i=0;i<LEN_TRHSMPL;i++){
      Serial.print((int8_t)rxPacket[i]);
      Serial.print(",");
    }
    Serial.println("******");      
  }

  if(topic == "perpet/SerialNumber/CMD"){
    if((payload.toInt())%3==0){
      Serial.println("shutdown");
    }
  }
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}


/*************************** Sketch Code ************************************/

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
// #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define SERVICE_UUID           "167d0001-d1ea-11ed-afa1-0242ac120002" // UART service UUID
#define CHARACTERISTIC_UUID_RX "167d0002-d1ea-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_TX "167d0003-d1ea-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_APID "167d0011-d1ea-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_APPW "167d0012-d1ea-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_MQTT "167d0013-d1ea-11ed-afa1-0242ac120002"
#define HEADERSIZE 8

char buf_eeprom[64];

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i],HEX);

        Serial.println();
        Serial.println("*********");
      }
    }
};

class MyCallbacks_APID: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("APID_Received: ");
      // String str = String(rxValue.substr(HEADERSIZE).c_str());
      String str = String(rxValue.c_str());
      AP_NAME.writeString(0, str);
      AP_NAME.commit();
      AP_NAME.get(0,buf_eeprom);
      Serial.println(String(buf_eeprom));

      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i],HEX);
      Serial.println();
      Serial.println("*********");
    }
  }
};

class MyCallbacks_APPW: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("APPW_Received: ");
      String str = String(rxValue.substr(HEADERSIZE).c_str());
      AP_PASSWORD.writeString(0, str);
      AP_PASSWORD.commit();
      AP_PASSWORD.get(0,buf_eeprom);
      Serial.println(String(buf_eeprom));

      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i],HEX);
      Serial.println();
      Serial.println("*********");
    }
  }
};

class MyCallbacks_MQTT: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("[BLE]MQTT_length: ");
      uint16_t len = rxValue[0];
      len= (len<<8)|rxValue[1];
      Serial.println(len);
      Serial.print("[BLE]MQTT_Received: ");
      String str = String(rxValue.substr(HEADERSIZE).c_str());
      MQTT_TOKEN.writeString(0, str);
      MQTT_TOKEN.commit();
      MQTT_TOKEN.get(0,buf_eeprom);
      Serial.println(String(buf_eeprom));


      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i],HEX);
      Serial.println();
      Serial.println("*********");
    }
  }
};

void writeFileBytes(fs::FS &fs, const char * path, const uint8_t * buf,uint8_t len){
    Serial.printf("Writing file: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.write(buf,len)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}
void appendFileBytes(fs::FS &fs, const char * path, const uint8_t * buf,uint8_t len){
    // Serial.printf("Appending file: %s\r\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.write(buf,len)){
        // Serial.println("- file appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}


const char* path = "/acc.txt";

// ref: (13:04) https://www.youtube.com/watch?v=JFDiqPHw3Vc&list=PL4s_3hkDEX_0YpVkHRY3MYHfGxxBNyKkj&index=100&t=590s
// clock | current wifi(mA) | current no wifi(mA) | Serial(baudrate) | i2c clock
//  240  |      157         |            69       |     115200       | 350kHz
//  160  |      131         |            46       |     115200       | 350kHz
//   80  |      119         |            32       |     115200       | 350kHz
//   40  |       82         |            18       |      57600       | 100kHz
//   20  |       77         |            13       |      28800       | 100kHz
//   10  |       74         |            11       |      14400       | 100kHz
void setup() {

  // Serial.begin(115200);
  Serial.begin(230400);

  
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }
    
  listDir(SPIFFS, "/", 0);
  // writeFile(SPIFFS, "/hello.txt", "Hello ");
  // appendFile(SPIFFS, "/hello.txt", "World!\r\n");
  // readFile(SPIFFS, "/hello.txt");
  // renameFile(SPIFFS, "/hello.txt", "/foo.txt");
  // testFileIO(SPIFFS, "/test.txt");
  deleteFile(SPIFFS, "/acc.txt");


  // Initialize EEPROM
  init_eeprom();

  INIT_FLAG.get(0,buf_eeprom);
  String init_flag=String(buf_eeprom);
  Serial.println("--flag--");
  Serial.println(init_flag);
  Serial.println("--------");

  if(init_flag!="EEPROM_INITIATED"){
  // Write: Variables ---> EEPROM stores
    DEV_NAME.writeString(0, Dev_name);
    DEV_NAME.commit();
    SERVER_IP.writeString(0, Server_ip);
    SERVER_IP.commit();
    SERVER_PORT.put(0, Server_port);
    SERVER_PORT.commit();
    AP_NAME.writeString(0, AP_id);
    AP_NAME.commit();
    AP_PASSWORD.writeString(0, AP_pw);
    AP_PASSWORD.commit();

    INIT_FLAG.writeString(0, "EEPROM_INITIATED");
    INIT_FLAG.commit();
    Serial.print("device name: ");   Serial.println(Dev_name);
    Serial.print("target server ip: ");   Serial.println(Server_ip);
    Serial.print("target server port: ");   Serial.println(Server_port);
    Serial.print("AP name: ");   Serial.println(AP_id);
    Serial.print("AP password: ");   Serial.println(AP_pw);
    Serial.println("EEPROM now initiated.");
    Serial.println("");
  }else{
    Serial.println("EEPROM already initiated!");
    Serial.println("");
  }
  // Clear variables
//  buf_eeprom[0]='\0';

  Serial.println("");
  DEV_NAME.get(0, buf_eeprom);
  Dev_name=String(buf_eeprom);
  SERVER_IP.get(0, buf_eeprom);
  Server_ip=String(buf_eeprom);
  SERVER_PORT.get(0, Server_port);
//  Server_port=String(buf_eeprom);
  AP_NAME.get(0,buf_eeprom);
  AP_id=String(buf_eeprom);
  AP_PASSWORD.get(0,buf_eeprom);
  AP_pw=String(buf_eeprom);

  // setDevInfo();
  // setDevParams();
  // setSensorParams();

  bool bBLEactivated;
  if(true){
    // Create the BLE Device
    BLEDevice::init("Perpet");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
    
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    BLEDescriptor *RxCharacteristic = new BLEDescriptor((uint16_t)0x2904);
    BLECharacteristic * pCharacteristicAPID = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_APID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pCharacteristicAPID->setCallbacks(new MyCallbacks_APID());

    BLECharacteristic * pCharacteristicAPPW = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_APPW,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pCharacteristicAPPW->setCallbacks(new MyCallbacks_APPW());

    BLECharacteristic * pCharacteristicMQTT = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_MQTT,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pCharacteristicMQTT->setCallbacks(new MyCallbacks_MQTT());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("[BLE]Waiting a client connection to notify...");
  }



  print_settings();

  delay(1000);
  Serial.println("connecting to AP");
  bool isAPconnected=false;
  // WiFi.begin(ssid, pass);
  for(int i=0;i<3;i++){
    isAPconnected=ConnectToRouter(AP_id.c_str(), AP_pw.c_str());
    if(isAPconnected){
      break;
    }
  }
  if(isAPconnected){
    Serial.println("AP connected");
  }else{
    Serial.println("AP not connected. Proceed anyway..");
  }
  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.

  MQTTclient.begin("jayutest.best", net);
  MQTTclient.onMessage(messageReceived);
  // MQTTclient.onMessageAdvanced(messageReceivedAdvanced);

  connect();



  int8_t accdum[90];
  Serial.println("writing data");
  // for(int j = 0 ; j < 30 ; j++){
  //   accdum[3*j]=(int8_t)(j+1);
  //   accdum[3*j+1]=(int8_t)(j+1);
  //   accdum[3*j+2]=-(int8_t)(j+1);
  // }
  writeFileBytes(SPIFFS, path, (uint8_t*)accdum,0);


  // for(int i = 0 ; i<60*1 ; i++){
  //   appendFileBytes(SPIFFS, path, (uint8_t*)accdum,90);
  //   Serial.println(i);
  // }
  // light_sleep_purpet();
}

void light_sleep_purpet(){
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  Serial.println("Going to sleep now");
  Serial.flush();
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  esp_light_sleep_start();
  Serial.println("WiFi turning on!");
  // WiFi.begin(ssid, pass);
  // Serial.println("WiFi turned on!");
}

// int8_t dummy[LEN_ACCSMPL]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
int8_t accbuf[90]; //3*1*30 Hz * 1 sec
int32_t prsbuf[80];   //4* 10 Hz * 2 sec
int16_t tempbuf[20]; // 2* 0.1 Hz * 100sec

uint32_t timestamp[3] = {2000,2000,2000};
uint32_t dt[3]={33,100,10000};
int8_t idx[3]={0,0,0};
uint16_t iter=0;

bool bViewerActive=false;
bool bPetActive=true;

void loop() {
      // ESP.restart();
  Serial.println("loop");

  MQTTclient.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!MQTTclient.connected()) {
    connect();
  }

  Serial.println("mqtt done");

  if(bViewerActive==true){
    // setCpuFrequencyMhz(80); //No BT/Wifi: 10,20,40 MHz, for BT/Wifi, 80,160,240MHz 
    //function - sensing

    // save sensor data
    // IMU: 30Hz, Alt: 10 Hz, RH: 10s, T: 10s
    timestamp[0]=millis(); //check it later
    if(millis()>timestamp[0]){
      timestamp[0]+=dt[0];
      // getSensorDataIMU();
      acc[idx[0]][0]=(int8_t)(idx[0]+1);
      acc[idx[0]][1]=(int8_t)(idx[0]+1);
      acc[idx[0]][2]=-(int8_t)(idx[0]+1);
      idx[0]++;
      if(idx[0]==LEN_ACCSMPL){
        MQTTclient.publish("perpet/SerialNumber/acc", (const char*)acc,LEN_ACCSMPL*3);
        idx[0]=0;
      }
    }     

    if(millis()>timestamp[1]){
      //sensor data sampling
      timestamp[1]+=dt[1];
      if(idx[1]==LEN_PRSSMPL){
        MQTTclient.publish("perpet/SerialNumber/prs", (const char*)prs,LEN_PRSSMPL*4);
      }
    }
    if(millis()>timestamp[2]){
      //sensor data sampling
      timestamp[2]+=dt[2];
      if(idx[2]==LEN_TRHSMPL){
        MQTTclient.publish("perpet/SerialNumber/trh", (const char*)trh,LEN_TRHSMPL*4);
      }
    }

  }else{
    Serial.println("owner:non-active");
    if(bPetActive==true){
      
      Serial.println("pet:active");
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      delay(10);
      setCpuFrequencyMhz(20); //No BT/Wifi: 10,20,40 MHz, for BT/Wifi, 80,160,240MHz 
      delay(10);
      Serial.println("appending data");

      uint32_t timestamp_dive = millis(); 
      idx[0]=2;
      while(true){
        //logging sensor data
        // IMU: 30Hz, Alt: 10 Hz, RH: 10s, T: 10s
        timestamp[0]=millis(); //check it later
        if(millis()>timestamp[0]){
          timestamp[0]+=dt[0];
          if(idx[0]==2){
            acc[0][0]=30;
            acc[0][1]=(int8_t)(iter>>8);
            acc[0][2]=(int8_t)iter;
            acc[1][0]=30;
            acc[1][1]=(int8_t)(iter>>8);
            acc[1][2]=(int8_t)iter;
            // Serial.println(iter);
            iter++;
          }
          // getSensorDataIMU();
          acc[idx[0]][0]=(int8_t)(idx[0]+1);
          acc[idx[0]][1]=(int8_t)(idx[0]+1);
          acc[idx[0]][2]=-(int8_t)(idx[0]+1);
          idx[0]++;
          if(idx[0]==LEN_ACCSMPL+2){
            appendFileBytes(SPIFFS, path, (uint8_t*)acc, LEN_ACCSMPL*3+6);
            idx[0]=2;
          }
        } 

        if(millis()-timestamp_dive>2*60*1000){
          break;
        }
      }

      setCpuFrequencyMhz(80); //No BT/Wifi: 10,20,40 MHz, for BT/Wifi, 80,160,240MHz 

      Serial.println("connecting to AP");
      bool isAPconnected=false;
      // WiFi.begin(ssid, pass);
      for(int i=0;i<3;i++){
        isAPconnected=ConnectToRouter(AP_id.c_str(), AP_pw.c_str());
        if(isAPconnected){
          break;
        }
      }
      if(isAPconnected){
        Serial.println("AP connected");
      }else{
        Serial.println("AP not connected. Proceed anyway..");
      }

      //transmitting the log
      if (!MQTTclient.connected()) {
        connect();
      }
      Serial.printf("Reading file: %s\r\n", path);
      File file = SPIFFS.open(path);
      if(!file || file.isDirectory()){
          Serial.println("- failed to open file for reading");
          return;
      }
      Serial.println("- read from file:");
      uint8_t buffer[LEN_ACCSMPL*3+6];
      // while(file.available()){
      if(file.available()){
        for(int i = 0; i<2*60; i++){
          file.seek(i*(LEN_ACCSMPL*3+6));
          file.read(buffer,LEN_ACCSMPL*3+6);
          // for(int i = 0 ; i < LEN_ACCBUF*3 ; i ++){
          //   Serial.print(buffer[i]);
          //   if(i%3==2){
          //     Serial.println();
          //   }else{
          //     Serial.print(",");
          //   }
          // }
          Serial.println(i);
          MQTTclient.publish("perpet/SerialNumber/acc", (const char*)buffer,LEN_ACCSMPL*3+6);
        }
      }
      file.close();

    }else{
      setCpuFrequencyMhz(20); //No BT/Wifi: 10,20,40 MHz, for BT/Wifi, 80,160,240MHz 
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
      Serial.println("Going to sleep now");
      Serial.flush();
      // WiFi.disconnect(true);
      // WiFi.mode(WIFI_OFF);
      esp_light_sleep_start();
      Serial.println("WiFi turning on!");
    }
  }

  if (deviceConnected) {
    pTxCharacteristic->setValue(&txValue, 1);
    pTxCharacteristic->notify();
    txValue++;
		delay(50); // bluetooth stack will go into congestion, if too many packets are sent
	}

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  

  // light_sleep_purpet();/
}
