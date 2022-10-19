#include <Arduino.h>
#include <SPI.h>
#define TYPE_MK10e
#include "EDMICmdLine.h"
// #include <lorawan.h>
// #define TYPE_SEND_LORA
// #if defined(TYPE_SEND_LORA)

// #include <Wire.h>
// #include <RTClib.h>
// #define csLora 5                 // PIN CS LORA
// const byte pinLED = LED_BUILTIN; // INDICATOR LED SERIAL
// const sRFM_pins RFM_pins = {
//   .CS = 5,
//   .RST = 26,
//   .DIO0 = 25,
//   .DIO1 = 32,
// }; // DEF PIN LORA
// byte lastTime;
// DateTime now;
// RTC_DS1307 rtc;
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// #define MODE_LORA 0 // LORA
// #endif
// // DEF PIN
// #define RXD2 16         // PIN RX2
// #define TXD2 17         // PIN TX2
#define PIN_LED_PROC 27 // PIN RX2
#define PIN_LED_WARN 15 // PIN TX2

// // MQTT CONF
// #define MQTT_USER_ID "anyone"
#if defined(ESP32)
EdmiCMDReader edmiread(Serial2,16,17);
#else
EdmiCMDReader edmiread(Serial2);
#endif
//  Var CONFIG DEFAULT
String serverName;
String customerID = "69696969";
String kwhID;
String kwhType = "MK10E";
const String channelId = "monKWH/";
String Port;
String userKey;
String apiKey;
String apid;
String ipAddres;
String ipTCP;
String local_portTCP;
String remote_portTCP;
String Gateway;
String DNS;
String netMask;
String dataProfile;
String devAddrLora = "260DB31D";
String nwkSKeyLora = "45789AEC3AA3D97D22602D0CDDC5BBA8";
String appSKeyLora = "3868C323B2A6AD96AB8BAFB7AB3B002C";
bool checkDHCP;
bool enableWifi;
bool enableLAN;
bool enableLora;
bool enableConverter;
bool factoryReset;
bool rebootDevice;
const unsigned int interval = 5000;
const unsigned int interval_cek_konek = 2000;
const unsigned int interval_record = 20000;
long previousMillis = 0;
long previousMillis1 = 0;
bool saveIPAddress;
bool saveIPSerial;

// DEF LORA CONF

// VAR MODE KIRIM
bool kwhReadReady = false;
bool kwhSend = false;

// VAR DATA KWH
float currentNow = 0;
float currentLast = 0;
float currentPrev = 0;



// bool loraConf()
// {
//   // if (!lora.init())
//   // {
//   //   //server.println("DEBUG: loraConf()");
//   //   //server.println("    LORA NOT DETECTED - WAIT 5 SEC");
//   //   delay(5000);
//   //   return false;
//   // }
//   lora.setDeviceClass(CLASS_C);
//   lora.setDataRate(SF7BW125);
//   lora.setChannel(MULTI);
//   lora.setNwkSKey(nwkSKeyLora.c_str());
//   lora.setAppSKey(appSKeyLora.c_str());
//   lora.setDevAddr(devAddrLora.c_str());

//   return true;
// }


void backgroundTask(void)
{

  edmiread.keepAlive();
  // if (enableLora)
  // lora.update();
  //  now = rtc.now();
  //  Serial.print(now.year(), DEC);
  //  Serial.print('/');
  //  Serial.print(now.month(), DEC);
  //  Serial.print('/');
  //  Serial.print(now.day(), DEC);
  //  Serial.print(" (");
  //  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  //  Serial.print(") ");
  //  Serial.print(now.hour(), DEC);
  //  Serial.print(':');
  //  Serial.print(now.minute(), DEC);
  //  Serial.print(':');
  //  Serial.print(now.second(), DEC);
  //  Serial.println();

}

void blinkWarning(int times)
{
  if (times == 0)
    digitalWrite(PIN_LED_WARN, 0);
  else
  {
    for (int x = 0; x <= times; x++)
    {
      static long prevMill = 0;
      if (((long)millis() - prevMill) >= 500)
      {
        prevMill = millis();
        digitalWrite(PIN_LED_WARN, !digitalRead(PIN_LED_WARN));
      }
    }
  }
}

void blinkProcces(int times)
{
  for (int x = 0; x <= times; x++)
  {
    digitalWrite(PIN_LED_PROC, !digitalRead(PIN_LED_PROC));
    delay(50);
    digitalWrite(PIN_LED_PROC, !digitalRead(PIN_LED_PROC));
    delay(50);
  }
}

void BackgroundDelay()
{
  if (millis() - previousMillis > interval_record)


  {
    kwhReadReady = true;
    edmiread.acknowledge();
    Serial.println("TIME TO READ");
    // previousMillis = millis();
    previousMillis = millis();
  }
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(PIN_LED_PROC, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  digitalWrite(PIN_LED_PROC, LOW);
  // if (!lora.init())
  // {
  //   Serial.println("RFM95 not detected");
  // }
  // loraConf();
  if (kwhID.length() == 0)
  {
    kwhID = edmiread.serialNumber();
    Serial.println(kwhID);
  }

} // setup

void loop()
{
  //  backgroundTask();
  if (kwhID.length() == 0)
  {
    kwhID = edmiread.serialNumber();
    Serial.println(kwhID);
  }

  edmiread.read_looping();
  EdmiCMDReader::Status status = edmiread.status();
  if (status == EdmiCMDReader::Status::Ready and kwhReadReady)
  {
    blinkWarning(LOW);
    blinkProcces(3);
    Serial.println("STEP_START");
    edmiread.step_start();
  }
  else if (status == EdmiCMDReader::Status::Finish)
  {
    char myStr[255];
    sprintf(myStr, "~*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s#",
            customerID,
            kwhID,
            String(edmiread.voltR(), 4).c_str(),
            String(edmiread.voltS(), 4).c_str(),
            String(edmiread.voltT(), 4).c_str(),
            String(edmiread.currentR(), 4).c_str(),
            String(edmiread.currentS(), 4).c_str(),
            String(edmiread.currentT(), 4).c_str(),
            String(edmiread.wattR() / 10.0, 4).c_str(),
            String(edmiread.wattS() / 10.0, 4).c_str(),
            String(edmiread.wattT() / 10.0, 4).c_str(),
            String(edmiread.pf(), 4).c_str(),
            String(edmiread.frequency(), 4).c_str(),
            String(edmiread.kVarh() / 1000.0, 2).c_str(),
            String(edmiread.kwhLWBP() / 1000.0, 2).c_str(),
            String(edmiread.kwhWBP() / 1000.0, 2).c_str(),
            String(edmiread.kwhTotal() / 1000.0, 2).c_str());
    Serial.println(myStr);
    Serial.println("SEND WITH LORA");
    // if (millis() - previousMillis1 > interval)
    // { 
    //   lora.sendUplink(myStr, strlen(myStr), 0, 1);
    //   lora.update();
    //   previousMillis1 = millis();
    // }

    blinkProcces(5);
    kwhReadReady = false;
    // digitalWrite(PIN_LED_PROC, LOW);
  }
  else if (status != EdmiCMDReader::Status::Busy)
  {
    Serial.println("BUSY");
    blinkWarning(HIGH);
  }
  

  // digitalWrite(LED_BUILTIN, LOW);
  BackgroundDelay();

} // loop
