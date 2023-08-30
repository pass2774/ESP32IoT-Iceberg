#include "serialUI.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("hello");
}

String readString;
// void parse(String str){
//   String temp = str.substring(0,str.indexOf(','));
//   Serial.println("1st data="+temp);
// }



void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available()>0){
    if(Serial.available()>0){
      char c = Serial.read();
      readString += c;
      // Serial.print(c);
      if(c == '\n'){
        Serial.println(readString);
        parse(readString,',');
        readString="";
      }
    }
  }
}
