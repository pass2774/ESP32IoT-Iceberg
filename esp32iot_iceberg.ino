/*
 * WebSocketClientSocketIOack.ino
 *
 *  Created on: 20.07.2019
 *
 */
#define HAS_SSL 0

#include <Arduino.h>

#include <WiFi.h>
//#include <WiFiMulti.h>
//#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include "SerialUI.h"
#include "Settings.h"
#include "SocketIOmanager.h"
#include "wifi_custom.h"
#include <ArduinoJson.h>


#include <SocketIOclient.h>
//#include <ArduinoJson.h>
//WiFiMulti WiFiMulti;
SocketIOclient socketIO;

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            Serial.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            Serial.printf("[IOc] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
        {
            char * sptr = NULL;
            int id = strtol((char *)payload, &sptr, 10);
            Serial.printf("[IOc] get event: %s id: %d\n", payload, id);
            if(id) {
                payload = (uint8_t *)sptr;
            }
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload, length);
            if(error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.c_str());
                return;
            }

            String eventName = doc[0];
            Serial.printf("[IOc] event name: %s\n", eventName.c_str());

            String str_buf;
            //parsing by socket tag
            if(eventName=="DevReset"){
              DynamicJsonDocument doc(1024);
              JsonArray array = doc.to<JsonArray>();
              array.add("DevResp");  // add evnet name
              // add payload (parameters) for the event
              JsonObject param1 = array.createNestedObject();
              param1["result"] = "Restarting the device..";
              serializeJson(doc, str_buf);
              socketIO.sendEVENT(str_buf);
              Serial.print("[Socket] transmit:");
              Serial.println(str_buf);
              ESP.restart();              
            }else if(eventName=="Greetings"){
              str_buf = GetJsonString_ClientType();
              socketIO.sendEVENT(str_buf);
              Serial.print("[Socket] transmit:");
              Serial.println(str_buf);

            }else if(eventName=="DevInfo"){
              str_buf = GetJsonString_DevInfo();
              socketIO.sendEVENT(str_buf);
              Serial.print("[Socket] transmit:");
              Serial.println(str_buf);

            }else if(eventName=="DevParam"){
              str_buf = GetJsonString_DevParams();
              socketIO.sendEVENT(str_buf);
              Serial.print("[Socket] transmit:");
              Serial.println(str_buf);
              
            }else if(eventName==""){
              
            }else if(eventName==""){
              
            }else if(eventName==""){
              
            }


            // Message Includes a ID for a ACK (callback)
            if(id) {
                // creat JSON message for Socket.IO (ack)
                DynamicJsonDocument docOut(1024);
                JsonArray array = docOut.to<JsonArray>();

                // add payload (parameters) for the ack (callback function)
                JsonObject param1 = array.createNestedObject();
                param1["now"] = millis();

                // JSON to String (serializion)
                String output;
                output += id;
                serializeJson(docOut, output);

                // Send event
                socketIO.send(sIOtype_ACK, output);
            }
        }
            break;
        case sIOtype_ACK:
            Serial.printf("[IOc] get ack: %u\n", length);
            break;
        case sIOtype_ERROR:
            Serial.printf("[IOc] get error: %u\n", length);
            break;
        case sIOtype_BINARY_EVENT:
            Serial.printf("[IOc] get binary: %u\n", length);
            break;
        case sIOtype_BINARY_ACK:
            Serial.printf("[IOc] get binary ack: %u\n", length);
            break;
        default:
            Serial.printf("[IOc] greet!!!: %s id: %d\n", payload, 123);
            break;
    }
}




void setup() {
  //Serial.begin(921600);
  Serial.begin(115200);

  //Serial.setDebugOutput(true);
//    Serial.setDebugOutput(true);

  Serial.println();
  Serial.println("Serial on");
  Serial.println("Welcome. Input 'help' to get information");
  delay(100);


  init_eeprom();
  char buf_eeprom[32];


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

  setDevInfo();
  setDevParams();
  setSensorParams();

  print_settings();

  delay(1000);
  Serial.println("connecting to AP");
  for(int i=0;i<3;i++){
    if(ConnectToRouter(AP_id.c_str(), AP_pw.c_str())){
      break;
    }
  }
  Serial.println("AP connected");

  bool isSSL=true;
  // server address, port and URL
  if(isSSL){// (SSL) https:// or wss://
    socketIO.beginSSL(Server_ip, Server_port, "/socket.io/?EIO=3"); //confirmed
  }else{ //http:// or ws://
//  socketIO.begin(Server_ip, 3000, "/socket.io/?EIO=4");
    socketIO.begin(Server_ip, 3000, "/socket.io/?EIO=3"); // maybe 3..?  
  }
  // event handler
  socketIO.onEvent(socketIOEvent);
};



unsigned long messageTimestamp = 0;
String readString;

void loop() {

  while(Serial.available()) {
    delay(1);  //delay to allow buffer to fill 
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    }
  }
  
  if (readString.length() >0) {
    Serial.println(readString); //see what was received
    parse_packet(readString);
    readString="";
  }
  
  socketIO.loop();
  uint64_t now = millis();

  if(now - messageTimestamp > 2000) {
      messageTimestamp = now;
      if(true){
        enroll_serviceProfile();
      }

      // creat JSON message for Socket.IO (event)
      DynamicJsonDocument doc(1024);
      JsonArray array = doc.to<JsonArray>();

      // add evnet name
      // Hint: socket.on('event_name', ....
//        array.add("event_name");
      array.add("msg-v0");//header:message-version-0 (test header)

      // add payload (parameters) for the event
      JsonObject param1 = array.createNestedObject();
      param1["now"] = (uint32_t) now;

      // JSON to String (serializion)
      String output;
      serializeJson(doc, output);
      // Print JSON for debugging
      Serial.print("transmitting:");
      Serial.println(output);
      // Send event
      socketIO.sendEVENT(output);

      String str_temp;
      str_temp = GetJsonString_example();
      socketIO.sendEVENT(str_temp);
      Serial.print("transmitting2:");
      Serial.println(str_temp);

//      str_temp = GetJsonString_DevInfo();
//      socketIO.sendEVENT(str_temp);
//      Serial.print("transmitting3:");
//      Serial.println(str_temp);

      main_task();
  }
}

void enroll_serviceProfile(void){
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("Start_Service");
  JsonObject root = array.createNestedObject();
  root["room"]="my-little-tiny-room";
  root["type"]="tsSensor";
  root["description"]="tsSensor";
  JsonObject dev_info = root.createNestedObject("contents");
  dev_info["sensor"] = "dummy sensor";

  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  socketIO.sendEVENT(str);
  Serial.println("ENROLL:");
  Serial.println(str);
  return;
}

void main_task(void){
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("msg-v0");
  JsonObject root = array.createNestedObject();

  JsonArray sensor_data[2];
  sensor_data[0] = root.createNestedArray("sensor_0");
  sensor_data[1] = root.createNestedArray("sensor_1");

  sensor_data[0].add(1);
  sensor_data[0].add(2);
  sensor_data[0].add(3);
  sensor_data[0].add(4);
  
  sensor_data[1].add(10);
  sensor_data[1].add(20);
  sensor_data[1].add(30);
  sensor_data[1].add(40);


  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  socketIO.sendEVENT(str);
  Serial.print("DATA:");
  Serial.println(str);
  return; 
}
