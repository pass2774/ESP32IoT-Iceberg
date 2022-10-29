#include "Settings.h"
//#define USE_SERIAL Serial


void print_settings(){
  Serial.println("--------Current settings--------");
  Serial.print("Device name: ");   Serial.println(Dev_name);
  Serial.print("Server ip: ");   Serial.println(Server_ip);
  Serial.print("Server port: ");   Serial.println(Server_port);
  Serial.print("AP name: ");   Serial.println(AP_id);
  Serial.print("AP password: ");   Serial.println(AP_pw);
  Serial.println("--------------------------------");
}

void parse_packet(String str){
  char buf_eeprom[32];
  String header,data;

  if(str.endsWith("\n")){
    str=str.substring(0,str.length()-1);
  }
  
  int i = str.indexOf(':');
  header = str.substring(0, i);
  data   = str.substring(i+1,str.length()); 

  Serial.print("header:");
  Serial.println(header);
  Serial.print("data:");
  Serial.println(data);

  if(header.equalsIgnoreCase("reset")){
    ESP.restart();
  }else if(header.equalsIgnoreCase("white")){
    INIT_FLAG.writeString(0, "\0");
    INIT_FLAG.commit();
    Serial.println("Input 'reset' to apply the result.(restart)");
    vTaskDelay(3000);    
  }else if(header.equalsIgnoreCase("help")){
    Serial.println("--------Available commands--------");
    Serial.println("[b]help");
    Serial.println("[b]reset");
    Serial.println("[b]white - Erase all memories in the EEPROM");
    Serial.println("[W]DeviceName");
    Serial.println("[W]ServerIP - Target server IP address");
    Serial.println("[W]ServerPORT");
    Serial.println("[W]APName - AP:Access Point (ex. WiFi router)");
    Serial.println("[W]APPassword");
    Serial.println("ex)");
    Serial.println("DeviceName:example_name");
    Serial.println("ServerIP:192.168.0.2");
    Serial.println("ServerPort:3000");
    Serial.println("APName:myRouterID");
    Serial.println("APPassword:myRouterPW");
    Serial.println("----------------------------------");
    Serial.println("Pausing for 5 seconds.");
    vTaskDelay(5000);
    Serial.println("");
    Serial.println("");
  }
  else if(header.equalsIgnoreCase("settings")){
  
    DEV_NAME.get(0, buf_eeprom);
    Dev_name=String(buf_eeprom);
    SERVER_IP.get(0, buf_eeprom);
    Server_ip=String(buf_eeprom);
    AP_NAME.get(0,buf_eeprom);
    AP_id=String(buf_eeprom);
    AP_PASSWORD.get(0,buf_eeprom);
    AP_pw=String(buf_eeprom);
    print_settings();
    vTaskDelay(5000);
  }else if(header.equalsIgnoreCase("DeviceName")){
    Serial.println("DeviceName has been updated.");
    Serial.print("New name: ");
    Serial.println(data);
    Dev_name=data;
    DEV_NAME.writeString(0, Dev_name);
    DEV_NAME.commit();
    Serial.println("Input reset to apply the result.(restart)");
    vTaskDelay(3000);    
    
  }else if(header=="ServerIP"){
    Serial.println("ServerIP has been updated");
    Serial.print("New IP address: ");
    Serial.println(data);
    Server_ip=data;
    SERVER_IP.writeString(0, Server_ip);
    SERVER_IP.commit();
    Serial.println("Input reset to apply the result.(restart)");
    vTaskDelay(3000);    
    
  }else if(header=="ServerPort"){
    Serial.println("ServerPort has been updated");
    Serial.print("New Port: ");
    Serial.println(data);
    Server_port=(uint16_t)data.toInt();
    SERVER_PORT.put(0, Server_port);
    SERVER_PORT.commit();
    Serial.println("Input reset to apply the result.(restart)");
    vTaskDelay(3000);    
    
  }else if(header=="APName"){
    Serial.println("APName has been updated");
    Serial.print("New WiFi AP name: ");
    Serial.println(data);
    AP_id=data;
    AP_NAME.writeString(0, AP_id);
    AP_NAME.commit();
    Serial.println("Input reset to apply the result.(restart)");
    vTaskDelay(3000);    
    
  }else if(header=="APPassword"){
    Serial.println("APPassword has been updated");
    Serial.print("New WiFi pw: ");
    Serial.print(data);
    AP_pw=data;
    AP_PASSWORD.writeString(0, AP_pw);
    AP_PASSWORD.commit();
    Serial.println("Input reset to apply the result.(restart)");
    vTaskDelay(3000);    
    
  }
  return;
}
