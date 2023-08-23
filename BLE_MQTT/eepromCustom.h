#ifndef __SETTINGS__
#define __SETTINGS__
#include "EEPROM.h"
//ESP32 by Espressif Systems Version 2.0.5. Other version may have different EEPROMClass methods

// EEPROMClass  INIT_FLAG("INIT_FLAG", 0x40);
// EEPROMClass  DEV_NAME("dev_name", 0x100);
// EEPROMClass  SERVER_IP("server_ip", 0x200);
// EEPROMClass  SERVER_PORT("server_port", 10);
// EEPROMClass  AP_NAME("ap_id", 0x100);
// EEPROMClass  AP_PASSWORD("ap_pw", 0x100);
EEPROMClass  INIT_FLAG("INIT_FLAG");
EEPROMClass  DEV_NAME("dev_name");
EEPROMClass  SERIAL_NUMBER("serial_number");
EEPROMClass  SERVER_IP("server_ip");
EEPROMClass  SERVER_PORT("server_port");
EEPROMClass  AP_NAME("ap_id");
EEPROMClass  AP_PASSWORD("ap_pw");
EEPROMClass  MQTT_TOKEN("mqtt_token");
EEPROMClass  SENSOR_PRS("sensor_prs");
EEPROMClass  SENSOR_TEMP("sensor_temp");
EEPROMClass  SENSOR_RH("sensor_rh");

EEPROMClass  SENSOR_ACC("sensor_acc");

//char* ssid = "Compass";
//char* password = "iepstt92";
String dev_name = "exampleDev";
String serial_number = "SN000-DEV-0010";
// String serial_number = "SNUMD-DEV-0001";
String server_addr = "192.168.0.3";
uint16_t server_port = 3000;
// String AP_id = "ANTS Place";
// String AP_pw = "ants@1681";
String AP_id = "JoonhwaHotSpot";
String AP_pw = "iepstt2774";
String topic_base = "perpet/"+serial_number;

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

char buf_eeprom[64];

bool init_eeprom(){
    if (!INIT_FLAG.begin(0x40)) {
    Serial.println("Failed to initialize EEPROM:INIT_FLAG");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!DEV_NAME.begin(0x100)) {
    Serial.println("Failed to initialize EEPROM:DEV_NAME");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!SERIAL_NUMBER.begin(0xFF)) {
    Serial.println("Failed to initialize EEPROM:SERIAL_NUMBER");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }  
  if (!SERVER_IP.begin(0x200)) {
    Serial.println("Failed to initialize EEPROM:SERVER_IP");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!SERVER_PORT.begin(0x0A)) {
    Serial.println("Failed to initialize EEPROM:SERVER_PORT");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  if (!AP_NAME.begin(0x100)) {
    Serial.println("Failed to initialize EEPROM:AP_NAME");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!AP_PASSWORD.begin(0x100)) {
    Serial.println("Failed to initialize EEPROM:AP_PASSWORD");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!MQTT_TOKEN.begin(512)) {
    Serial.println("Failed to initialize EEPROM:MQTT_TOKEN");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!SENSOR_ACC.begin(8*1024)) {
    Serial.println("Failed to initialize EEPROM:SENSOR_ACC0");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  return true;
}

void eepromSetup_custom(){
  INIT_FLAG.get(0,buf_eeprom);
  String init_flag=String(buf_eeprom);
  Serial.println("--flag--");
  Serial.println(init_flag);
  Serial.println("--------");

  // if(init_flag!="EEPROM_INITIATED"){
  if(true){
  // Write: Variables ---> EEPROM stores
    DEV_NAME.writeString(0, dev_name);
    DEV_NAME.commit();
    SERIAL_NUMBER.writeString(0, serial_number);
    SERIAL_NUMBER.commit();
    SERVER_IP.writeString(0, server_addr);
    SERVER_IP.commit();
    SERVER_PORT.put(0, server_port);
    SERVER_PORT.commit();
    AP_NAME.writeString(0, AP_id);
    AP_NAME.commit();
    AP_PASSWORD.writeString(0, AP_pw);
    AP_PASSWORD.commit();

    INIT_FLAG.writeString(0, "EEPROM_INITIATED");
    INIT_FLAG.commit();
    Serial.print("device name: ");   Serial.println(dev_name);
    Serial.print("target server addr: ");   Serial.println(server_addr);
    Serial.print("target server port: ");   Serial.println(server_port);
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
  dev_name=String(buf_eeprom);
  SERIAL_NUMBER.get(0, buf_eeprom);
  serial_number=String(buf_eeprom);
  SERVER_IP.get(0, buf_eeprom);
  server_addr=String(buf_eeprom);
  SERVER_PORT.get(0, server_port);
//  Server_port=String(buf_eeprom);
  AP_NAME.get(0,buf_eeprom);
  AP_id=String(buf_eeprom);
  AP_PASSWORD.get(0,buf_eeprom);
  AP_pw=String(buf_eeprom);

  // setDevInfo();
  // setDevParams();
  // setSensorParams();
}


void setDevInfo(){
  devInfo.id="PERPET0000";
  devInfo.pw="0000";
  devInfo.description="PERPET";
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
