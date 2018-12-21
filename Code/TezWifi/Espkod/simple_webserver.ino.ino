
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

#define USE_SERIAL Serial

ESP8266WiFiMulti WiFiMulti;

void setup() {

  USE_SERIAL.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("CTNKYA", "champselysees");

}

void loop() {
   String data=receivelastmessage();
   
   //Serial.println(data);
 if(data=="guncelle")
 { 
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    HTTPClient http;
    http.begin("http://firmwareguncelle.000webhostapp.com/user_app.bin");  
    int httpCode = http.GET();
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
       int len = http.getSize();
       
       uint8_t buff[128] = { 0 };
        WiFiClient * stream = http.getStreamPtr();
        while (http.connected() && (len > 0 || len == -1)) {
          size_t size = stream->available();

          if (size) {
            
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));  
           
          
            USE_SERIAL.write(buff, c);
            

            if (len > 0) {
              len -= c;
            }
          }
          //delay(1);
        }

      }
    } else {
      USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }
  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();
  delay(30000);
 }
}

String receivelastmessage()
{
  String lastmessage="";
  if(WiFi.status()==WL_CONNECTED)
  {
    HTTPClient http;
    String url="http://firmwareguncelle.000webhostapp.com/datastorage.txt";
    http.begin(url);
    http.addHeader("Content-Type","text/plain");
    int httpCode=http.GET();
    String data=http.getString();
    lastmessage=getlastline(data);
    http.end();
  }
  else
  {
    lastmessage="";
  }
  return lastmessage;
}

String getlastline(String str)
{
  String s="";
  int len=str.length();
  for (int i=len-2;i>=0;i--)
  {
    if (str[i]=='\n')
    {
      break;
    }
    else
    {
      s=s+str[i];
    }
  }
  String rs="";
  for (int i=s.length()-1;i>=0;i--)
  {
    rs=rs+s[i];
  }
  return rs;
}



