#ifndef __SETTINGS__
#define __SETTINGS__
#include "EEPROM.h"

EEPROMClass  INIT_FLAG("INIT_FLAG", 0x40);
EEPROMClass  DEV_NAME("dev_name", 0x100);
EEPROMClass  SERVER_IP("server_ip", 0x200);
EEPROMClass  SERVER_PORT("server_port", 10);
EEPROMClass  AP_NAME("ap_id", 0x100);
EEPROMClass  AP_PASSWORD("ap_pw", 0x100);
//char* ssid = "Compass";
//char* password = "iepstt92";
String Dev_name = "exampleDev";
String Server_ip = "192.168.0.3";
uint16_t Server_port = 3000;
String AP_id = "Compass";
String AP_pw = "iepstt92";

struct Sensor{
  String name;
  uint32_t sampling_interval;
  uint16_t buffer_size;
  bool enable;
  bool en_avg_filter;
  //what else..?
};

Sensor sensor[2];

struct DevInfo{
  String id;
  String pw;
  String description;
  bool b_TimeSeriesData;
  bool b_MediaData;
};
DevInfo devInfo;


struct DevParams{
  int32_t condition_1;
  uint32_t condition_2;
  bool condition_3;
};
DevParams devParams;


bool init_eeprom(){
    if (!INIT_FLAG.begin(INIT_FLAG.length())) {
    Serial.println("Failed to initialize EEPROM:INIT_FLAG");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!DEV_NAME.begin(DEV_NAME.length())) {
    Serial.println("Failed to initialize EEPROM:DEV_NAME");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!SERVER_IP.begin(SERVER_IP.length())) {
    Serial.println("Failed to initialize EEPROM:SERVER_IP");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!SERVER_PORT.begin(SERVER_PORT.length())) {
    Serial.println("Failed to initialize EEPROM:SERVER_PORT");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  if (!AP_NAME.begin(AP_NAME.length())) {
    Serial.println("Failed to initialize EEPROM:AP_NAME");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!AP_PASSWORD.begin(AP_PASSWORD.length())) {
    Serial.println("Failed to initialize EEPROM:AP_PASSWORD");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  return true;
}



void setDevInfo(){
  devInfo.id="dev0000";
  devInfo.pw="0000";
  devInfo.description="potentiostat";
  devInfo.b_TimeSeriesData=true;
  devInfo.b_MediaData=false;
}
void setDevParams(){
  devParams.condition_1=1;
  devParams.condition_2=2;
  devParams.condition_3=true;
}
void setSensorParams(){
  sensor[0].name = "potential/V";
  sensor[0].sampling_interval = 10;
  sensor[0].buffer_size = 10;
  sensor[0].enable = true;
  sensor[0].en_avg_filter = false;

  sensor[1].name = "current/mA";
  sensor[1].sampling_interval = 10;
  sensor[1].buffer_size = 10;
  sensor[1].enable = true;
  sensor[1].en_avg_filter = false;

  //what about table-form data?
  //ex)
  //  t: time, x: potential, y: current, ....
}

#endif
