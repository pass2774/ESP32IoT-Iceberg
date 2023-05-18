
#include <WiFi.h>
#include <WiFiClient.h>
//#include <WebSocketClient.h>
//WebSocketClient webSocketClient;
WiFiClient client;


//char path[] = "/";
//char host[] = "192.168.101.178";    // 웹소켓 서버 주소

int ws_update_interval = 1000;
uint64_t ws_serverTime_ms = 0;
bool ConnectToRouter(const char* routerID,const char* routerPW){
    //connecting to wifi router
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(routerID);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(routerID, routerPW);
//    WiFi.begin("Compass", "iepstt92");

//    while (WiFi.status() != WL_CONNECTED) {
//        delay(2000);
//        Serial.print(".");
//    }
    for (int i=0;WiFi.status() != WL_CONNECTED;i++) {
        delay(500);
        Serial.print(".");
        if(i>=10){
          WiFi.disconnect();
          Serial.println("WiFi connection is unable. Retrying..");
          WiFi.begin(routerID, routerPW);
//          i=0;
          return false;
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());  

    delay(2000);  
    return true;
}

void ReConnectToRouter(char* routerID,char* routerPW){
    //connecting to wifi router
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(routerID);
    delay(100);
    WiFi.disconnect();
    WiFi.begin(routerID, routerPW);

    for (int i=0;WiFi.status() != WL_CONNECTED;i++) {
        delay(300);
        Serial.print(".");
        if(i>=10){
          WiFi.disconnect();
          Serial.println("WiFi connection is unable. Retrying..");
          WiFi.begin(routerID, routerPW);
          i=0;
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());  
    delay(2000);  
    return;
}

bool Check_RouterConnection(){
  return (WiFi.status() == WL_CONNECTED);
}

//
//bool Check_WebSocketServerConnection(){
//  return webSocketClient.handshake(client);
//}
//
//
//void ConnectToWebSocketServer(char* hostIP, int hostPort, char* hostPath){
//
//    // Connecting to server
//    if (client.connect(hostIP, hostPort)) {
//        Serial.println("Connected");
//    } else {
//        Serial.println("Connection failed.");
//    }
////    delay(1000); 
//    delay(100); 
//
//    webSocketClient.path = hostPath;
//    webSocketClient.host = hostIP;
// 
//    if (webSocketClient.handshake(client)) {
//        Serial.println("Handshake successful");
//    } else {
//        Serial.println("Handshake failed.");
//    }
//  return;
//}
//
//
//bool toggle[3]={false,false,false};
//int ws_dac_output[4]={0,0,0,0};
//bool ws_dac_relay_enable[4]={false, false, false, false };
//bool parseWebSocketData(String str){
//  String str2;
//  int idx_comma = 0;
//  
//  while(idx_comma>=0){
////    idx_comma = str.indexOf(",");
////    int len=str.length();
////    String str_temp=str.substring(0,idx_comma);
////    str2=str.substring(idx_comma+1,len);
////    str=str2;
//
//    String str_temp=str;
//    Serial.print("strTemp=");
//    Serial.print(str_temp);
//
//    int len_temp=str_temp.length();
//    int idx_value=str_temp.indexOf(",",0); //myString.indexOf(val, from)
//    String header=str_temp.substring(0,idx_value);
//    String data_content=str_temp.substring(idx_value+1,len_temp);
////    Serial.print("!str= ");
////    Serial.println(str);
////    Serial.print("[!!]cmd name=[");
////    Serial.print(cmd_name);
////    Serial.print("], cmd value=[");
////    Serial.print(cmd_value);
////    Serial.println("]");
//    if(header.equals("SVTms")){ //server time
//        ws_serverTime_ms=0;
//        for (int i = 0; data_content[i] != '\0'; ++i){
//          ws_serverTime_ms = ws_serverTime_ms*10 + (data_content[i] - '0');
//        }
//        return true;
////        ws_serverTime_ms = data_content.toInt(); 
//    }if(header.equals("toggle0")){
//        if(data_content.toInt()==1){
//          toggle[0]=true;
//          Serial.println("toggle[0]=true");
//        }else{
//          toggle[0]=false;
//          Serial.println("toggle[0]=true");
//        }
//    }else if(header.equals("update_interval")){
//        ws_update_interval = data_content.toInt(); 
//        Serial.print("update_interval=");
//        Serial.println(ws_update_interval);
//    }else if(header.equals("dac_output0")){
//        ws_dac_output[0] = data_content.toInt(); 
//        Serial.print("dac_output[0]=");
//        Serial.println(ws_dac_output[0]);
//    }else if(header.equals("dac_relay_enable1")){
//        if(data_content.toInt()==1){
//          ws_dac_relay_enable[1]=true;
//          Serial.println("dac_relay_enable[1]=true");
//        }else{
//          ws_dac_relay_enable[1]=false;
//          Serial.println("dac_relay_enable[1]=false");
//        }
//    }
//  }
//  return true;
//}
