#ifndef __SOCKETIOMANAGER__
#define __SOCKETIOMANAGER__

#include <ArduinoJson.h>
#include "Settings.h"
String GetJsonString_ClientType(void){
   // creat JSON message for Socket.IO (event)
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("ClientTypeDeclaration");
  JsonObject client_type = array.createNestedObject();
  client_type["client_type"] = "CLIENT_DEVICE";
  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  return str; 
}

String GetJsonString_DevInfo(void){
  // creat JSON message for Socket.IO (event)
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("dev_info");
  JsonObject dev_info = array.createNestedObject();
  dev_info["id"] = devInfo.id;
  dev_info["pw"] = devInfo.pw;
  dev_info["description"] = devInfo.description;
  dev_info["TimeSeriesData"] = devInfo.b_TimeSeriesData;
  dev_info["MediaData"] = devInfo.b_MediaData;
  JsonObject sensor_list = dev_info.createNestedObject("Sensors");
  sensor_list["sensor0"] = sensor[0].name;
  sensor_list["sensor1"] = sensor[1].name;
  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  return str;
}
String GetJsonString_DevParams(void){
  // creat JSON message for Socket.IO (event)
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("dev_params");
  JsonObject dev_params = array.createNestedObject();
  dev_params["condition1"] = devParams.condition_1;
  dev_params["condition2"] = devParams.condition_2;
  dev_params["condition3"] = devParams.condition_3;

  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  return str;
}
//example - object in object, Nested object & Nested array
String GetJsonString_example(void){
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("msg-v0");
  JsonObject root = array.createNestedObject();
  JsonObject dev_info = root.createNestedObject("dev_info");
  dev_info["id"] = devInfo.id;
  dev_info["pw"] = devInfo.pw;
  dev_info["description"] = devInfo.description;
  dev_info["TimeSeriesData"] = devInfo.b_TimeSeriesData;
  dev_info["MediaData"] = devInfo.b_MediaData;
  
  JsonObject dev_params = root.createNestedObject("dev_params");
  dev_params["condition1"] = devParams.condition_1;
  dev_params["condition2"] = devParams.condition_2;
  dev_params["condition3"] = devParams.condition_3;
  
  JsonArray sensor_data = root.createNestedArray("sensor_1");
  sensor_data.add(1);
  sensor_data.add(2);
  sensor_data.add(3);
  sensor_data.add(4);
  
  // JSON to String (serializion)
  String str;
  serializeJson(doc, str);
  return str; 
}




#endif
