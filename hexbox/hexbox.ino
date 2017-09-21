/*  Hex Box 0.0.1
 *  Author: Timothy Garcia (http://timothygarcia.ca)
 *  Date: September 2017
 *  
 *  Description:
 *  Serves sensor data in JSON format access via http://hexbox.local/tcs34725.json
 *  Writes sensor data to SD card (not formatted, debug purposes only)
 *  Sets device time
 *  
 *  Hardware Used:
 *  Adafruit ESP8266 Huzzah / Adafruit Adalogger Featherwing RTC+SD / Adafruit TCS34725 Colour Sensor
 *  
 *  Extra Credits:
 *  Adafruit Industries www.adafruit.com
 *  ductsoup - experimental wrapper class that implements the improved lux and color temperature from TAOS and a basic autorange mechanism.
 *  
 *  License: This code is public domain. You can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation.  <http://www.gnu.org/licenses/>.
 *  Certain libraries used may be under a different license.
*/ 

#include <Wire.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "Adafruit_TCS34725.h"
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>

// SD Setup
Sd2Card card;
SdVolume volume;
SdFile root;
File configFile;
const int chipSelect = 15;

// RTC Setup
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String content;

// Wireless Setup
#define WLAN_SSID     "SSID"
#define WLAN_PASSWORD "PASSWORD"
#define DEVICE_NAME "hexbox"
#define VERSION_NUMBER "0.0.1"

// some magic numbers for this device from the DN40 application note
#define TCS34725_R_Coef 0.136 
#define TCS34725_G_Coef 1.000
#define TCS34725_B_Coef -0.444
#define TCS34725_GA 1.0
#define TCS34725_DF 310.0
#define TCS34725_CT_Coef 3810.0
#define TCS34725_CT_Offset 1391.0

ESP8266WebServer server(80);

unsigned long previousMillis = 0;
const long interval = 2000;

// Autorange class for TCS34725
class tcs34725 {
public:
  tcs34725(void);

  boolean begin(void);
  void getData(void);  

  boolean isAvailable, isSaturated;
  uint16_t againx, atime, atime_ms;
  uint16_t r, g, b, c;
  uint16_t ir; 
  uint16_t r_comp, g_comp, b_comp, c_comp;
  uint16_t saturation, saturation75;
  float cratio, cpl, ct, lux, maxlux;
  
private:
  struct tcs_agc {
    tcs34725Gain_t ag;
    tcs34725IntegrationTime_t at;
    uint16_t mincnt;
    uint16_t maxcnt;
  };
  static const tcs_agc agc_lst[];
  uint16_t agc_cur;

  void setGainTime(void);  
  Adafruit_TCS34725 tcs;    
};
//
// Gain/time combinations to use and the min/max limits for hysteresis 
// that avoid saturation. They should be in order from dim to bright. 
//
// Also set the first min count and the last max count to 0 to indicate 
// the start and end of the list. 
//
const tcs34725::tcs_agc tcs34725::agc_lst[] = {
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS,     0, 47566 },
  { TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS,  3171, 63422 },
  { TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15855, 63422 },
  { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_2_4MS,   248,     0 }
};
tcs34725::tcs34725() : agc_cur(0), isAvailable(0), isSaturated(0) {
}

// initialize the sensor
boolean tcs34725::begin(void) {
  tcs = Adafruit_TCS34725(agc_lst[agc_cur].at, agc_lst[agc_cur].ag);
  if ((isAvailable = tcs.begin())) 
    setGainTime();
  return(isAvailable);
}

// Set the gain and integration time
void tcs34725::setGainTime(void) {
  tcs.setGain(agc_lst[agc_cur].ag);
  tcs.setIntegrationTime(agc_lst[agc_cur].at);
  atime = int(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);  
  switch(agc_lst[agc_cur].ag) {
  case TCS34725_GAIN_1X: 
    againx = 1; 
    break;
  case TCS34725_GAIN_4X: 
    againx = 4; 
    break;
  case TCS34725_GAIN_16X: 
    againx = 16; 
    break;
  case TCS34725_GAIN_60X: 
    againx = 60; 
    break;
  }        
}

// Retrieve data from the sensor and do the calculations
void tcs34725::getData(void) {
  // read the sensor and autorange if necessary
  tcs.getRawData(&r, &g, &b, &c);
  while(1) {
    if (agc_lst[agc_cur].maxcnt && c > agc_lst[agc_cur].maxcnt) 
      agc_cur++;
    else if (agc_lst[agc_cur].mincnt && c < agc_lst[agc_cur].mincnt)
      agc_cur--;
    else break;

    setGainTime(); 
    delay((256 - atime) * 2.4 * 2); // shock absorber
    tcs.getRawData(&r, &g, &b, &c);
    break;    
  }

  // DN40 calculations
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
  r_comp = r - ir;
  g_comp = g - ir;
  b_comp = b - ir;
  c_comp = c - ir;   
  cratio = float(ir) / float(c);

  saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
  saturation75 = (atime_ms < 150) ? (saturation - saturation / 4) : saturation;
  isSaturated = (atime_ms < 150 && c > saturation75) ? 1 : 0;
  cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF); 
  maxlux = 65535 / (cpl * 3);

  lux = (TCS34725_R_Coef * float(r_comp) + TCS34725_G_Coef * float(g_comp) + TCS34725_B_Coef * float(b_comp)) / cpl;
  ct = TCS34725_CT_Coef * float(b_comp) / float(r_comp) + TCS34725_CT_Offset;
}

void saveConfig(String jsonString) {
  DateTime now = rtc.now();
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  configFile = SD.open(String(now.unixtime()) + "_log.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (configFile) {
    content += "Writing...";
    configFile.println(jsonString);
    // close the file:
    configFile.close();
    content += "done.";
  } else {
    // if the file didn't open, print an error:
    content += "error opening .txt";
  }
}

tcs34725 rgb_sensor;

void setup(void) {
  Serial.begin(115200);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  content += "Initializing SD card...";

  if (!SD.begin(chipSelect)) {
    content += "initialization failed. Things to check:";
    content += "* is a card inserted?";
    content += "* is your wiring correct?";
    content +="* did you change the chipSelect pin to match your shield or module?";
    return;
  } else {
    content += "Wiring is correct and a card is present.";
  }
  
  delay(10);
  
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  
  rgb_sensor.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  
  if (!MDNS.begin(DEVICE_NAME)) {
    Serial.println("Error setting up MDNS responder!");
    while(1) { 
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  
  pinMode(2, OUTPUT);
  analogWrite(2, 0); // @gremlins Bright light, bright light!

  //server.on("/save", saveConfig);
  server.on("/about.json", [](){
        DateTime now = rtc.now();

        StaticJsonBuffer<128> jsonBuffer;
        JsonObject& about = jsonBuffer.createObject();
        about["device_name"] = DEVICE_NAME;
        about["device_time"] = now.unixtime();
        about["device_software_version"] = VERSION_NUMBER;
        
        String jsonString;
        about.printTo(jsonString);

        Serial.println(jsonString);
        server.send(200, "application/json", jsonString);
    });
    
   server.on("/tcs34725.json", [](){
        unsigned long currentMillis = millis();
        DateTime now = rtc.now();

          if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;
              analogWrite(2, 100);
              rgb_sensor.getData();
              analogWrite(2, 0);
              delay(2000);
          }
        

        StaticJsonBuffer<717> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root["device_name"] = DEVICE_NAME;
        root["device_time"] = now.unixtime();
        //root["time"] = time_here_goes;

        JsonObject& raw = root.createNestedObject("raw");
        raw["red"] = rgb_sensor.r;
        raw["green"] = rgb_sensor.g;
        raw["blue"] = rgb_sensor.b;        
        raw["clear"] = rgb_sensor.c;

        JsonObject& compensated = root.createNestedObject("compensated");
        byte R = rgb_sensor.r_comp,G = rgb_sensor.g_comp,B = rgb_sensor.b_comp;
        char hex[7] = {0};
        sprintf(hex,"%02X%02X%02X",R,G,B); 
        compensated["red"] = rgb_sensor.r_comp;
        compensated["green"] = rgb_sensor.g_comp;
        compensated["blue"] = rgb_sensor.b_comp;        
        compensated["clear"] = rgb_sensor.c_comp;
        compensated["hex"] = hex;
 

        JsonObject& attributes = root.createNestedObject("attributes");
        attributes["ir"] = rgb_sensor.ir;
        attributes["cratio"] = rgb_sensor.cratio;
        attributes["saturation"] = rgb_sensor.saturation;        
        attributes["saturation75"] = rgb_sensor.saturation75;
        attributes["isSaturated"] = rgb_sensor.isSaturated;
        attributes["cpl"] = rgb_sensor.cpl;
        attributes["maxlux"] = rgb_sensor.maxlux;
        attributes["lux"] = rgb_sensor.lux;
        attributes["ct"] = rgb_sensor.ct;
        attributes["gain"] = rgb_sensor.againx;
        attributes["timems"] = rgb_sensor.atime_ms;
        attributes["atime"] = rgb_sensor.atime;
        
        String jsonString;
        root.printTo(jsonString);

        Serial.println(jsonString);
        server.send(200, "application/json", jsonString);
        saveConfig(jsonString);
    });

   server.begin();
   Serial.println("HTTP server started! Waiting for clients!");
   MDNS.addService("http", "tcp", 80);
}

void loop(void) {
  server.handleClient();
}
