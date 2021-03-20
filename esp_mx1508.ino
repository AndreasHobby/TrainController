
/* (c) Andreas' Hobby
 * (Lego-)TrainController via MX1508 motor driver v1.3
 * Sketch is controlling up to 2 motors via mx1508 driver boards and an ESP8266/ESP32 device 
 *  - controllable via HTTP request (implemented), MQTT possible to adapt
 * Based on examples from Neil Kolban and chegewara
 */

#if defined(ESP8266)
#include <ESP8266WiFi.h>          //install via Boardverwalter: http://arduino.esp8266.com/stable/package_esp8266com_index.json / https://dl.espressif.com/dl/package_esp32_index.json
#else
#include <WiFi.h>                 
#include <analogWrite.h>                  //url=https://github.com/ERROPiX/ESP32_AnalogWrite
#endif
#include <ArduinoOTA.h>                   //url=https://github.com/jandrassy/ArduinoOTA
#include <ESPAsyncWebServer.h>            //url=https://github.com/me-no-dev/ESPAsyncWebServer
#include "PasswordSettings.h"             //define ssid and password String in  external header file
//*****************************************************************************************************************
// ** WLAN declarations **
String IP = "";                           //stores current IP address
AsyncWebServer server(80);                //stores webserver data
//*****************************************************************************************************************
// ** PWM declarations **
#define MOTOR_MAX                   2      //define number of motors to be controlled (can be adapted as long as compatible outputs are free)
int Pin1[MOTOR_MAX] = {D1, D1};            //define pin1 for connection to mx1508 driver(s)
int Pin2[MOTOR_MAX] = {D2, D2} ;           //define pin2 for connection to mx1508 driver(s)
int MotorSpeed[MOTOR_MAX];                 //stores the current motor speed (range: -255 .. 0 .. +255)
//*****************************************************************************************************************
// ** WATCHDOG declarations **
#define Config_Watchdog             true   //configure watchdog feature
#define Config_WatchdogTimeoutTime  5000   //define watchdog timeout in Milliseconds
bool WatchdogState[MOTOR_MAX];             //stores state of watchdog
unsigned long WatchdogTimeout[MOTOR_MAX];  //stores time stamp the watchdog will stop the motor if no request is read in
//*****************************************************************************************************************
// ** voltage measuring declarations **
#define Config_VoltageReadInterval  60000  //define voltage read interval in Milliseconds
#define PinVoltage                  A0     //define pin to use for measure voltage via ADC
float AkkuVoltage = 0;                     //stores the current measured Akku voltage (Vin)
unsigned long NextVoltageRead = 0;         //stores time stamp the voltage will be read next time
//*****************************************************************************************************************
//*****************************************************************************************************************
// server callbacks
void CALLBACK_notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}
void CALLBACK_handleRoot(AsyncWebServerRequest *request) {
  String head = "<!DOCTYPE HTML><html><head><title>ESP Input Form</title>\
  <meta name='viewport' content='width=device-width, initial-scale=1'></head><body>\
  <form action='/get'>Motor: <input type='text' name='Motor' value='"+String(0)+"'><input type='submit' value='Submit'></form><br>\
  RSSI: "+String(WiFi.RSSI())+"\
  AkkuVoltage: "+String(AkkuVoltage/100)+"\
  MotorSpeed: "+String(MotorSpeed[0])+"\
  <br\>\
</body></html>";
 request->send(200, "text/html", head); //Send web page
}
void CALLBACK_handleGetRequest(AsyncWebServerRequest *request) {
    int input = 0;                          //stopping the motor in case of another request is received
    int channel = 0;                        //stopping the motor in case of another request is received
    if (request->hasParam("Motor")) {
      String inputString = request->getParam("Motor")->value();
      input = inputString.toInt();
      Serial.println("data recevied: " + String(input));
    }
    if (request->hasParam("Channel")) {
      String inputString = request->getParam("Channel")->value();
      channel = inputString.toInt();
      Serial.println("data recevied: " + String(channel));
    }
    FUN_doControl(channel, input);          //request motor control
    CALLBACK_handleRoot(request);
}
//*****************************************************************************************************************
// init wifi and setup OTA
void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
  }
  IP = IpAddress2String(WiFi.localIP());
  Serial.println("Wifi: Connected, IP: " + IP);

  // default setting of OTA is no auth and Port=8266
  // Hostname defaults to esp8266/ESP32-[ChipID]
  //ArduinoOTA.setHostname("");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) { Serial.println("Auth Failed"); } else if (error == OTA_BEGIN_ERROR) { Serial.println("Begin Failed"); } else if (error == OTA_CONNECT_ERROR) { Serial.println("Connect Failed"); }
    else if (error == OTA_RECEIVE_ERROR) { Serial.println("Receive Failed"); } else if (error == OTA_END_ERROR) { Serial.println("End Failed"); };
  });                                      //OTA show error
  ArduinoOTA.begin();                      //OTA start

  //setup callbacks for simple web server and start it
  server.on("/", CALLBACK_handleRoot);
  server.on("/get", HTTP_GET, CALLBACK_handleGetRequest);
  server.onNotFound(CALLBACK_notFound);
  server.begin();
};

//*****************************************************************************************************************
// stop motor of given channel
void FUN_doStop(int channel) {
  WatchdogState[channel] = false;          //reset watchdog state
  MotorSpeed[channel] = 0;
  analogWrite(Pin1[channel], 0);           //set both outputs to low level to stop motor
  analogWrite(Pin2[channel], 0);           //set both outputs to low level to stop motor
}
//*****************************************************************************************************************
// power motor of given channel by given value (range -255 .. 0 .. +255)
void FUN_doControl(int channel, int power) {
  if (power == 0) {
    FUN_doStop(channel);
  } else {
    WatchdogState[channel] =              Config_Watchdog;            //enable watchdog to auto stop motor in case of connection fault
    WatchdogTimeout[channel] = millis() + Config_WatchdogTimeoutTime; //calculate next time stamp (when the time is reached without an update (new request) the motor will be stopped
    MotorSpeed[channel] = power;
#if defined(ESP8266)
    power = power*4;
#endif    
    if (power > 0) {
      analogWrite(Pin1[channel], power);
      analogWrite(Pin2[channel], 0);
    } else {
      analogWrite(Pin1[channel], 0);
      analogWrite(Pin2[channel], -power);                             //negative value means that the polarity of the motor needs to be shifted (here we switch the pins); negative values needs to be inverted
    }
  }
}
//*****************************************************************************************************************
// watchdog: checks running time of motor after last request, if time is exceeding the motor will be stopped
void loop_Watchdog() {
  for (int i=0; i < MOTOR_MAX; i++){
    if(WatchdogState[i]) {                      
      if (millis() > WatchdogTimeout[i]){
        FUN_doStop(i);                     //stop motor in case of timeout (no new control request was received after given time
      } 
    }
  }
}
//*****************************************************************************************************************
// read analog voltage function; returns voltage in [V]
void loop_AkkuVoltageRead() {  
  if (millis() > NextVoltageRead){
    NextVoltageRead = millis() + Config_VoltageReadInterval;
    int AkkuVoltageRaw = analogRead(PinVoltage);  //read the analog signal
    AkkuVoltage = 100*AkkuVoltageRaw*0.00625;
    //Pin connected to GND and Vin with 10k Ohm resistor each; constant was calulated from measurement of value 660 at 4.13V (means: ~2.06V at Pin level) => factor=4.13/660
  }
}
//*****************************************************************************************************************
// init MotorDriverPWM
void initMotorDriver() {
  #if defined(ESP8266)
  analogWriteFreq(19500); //PWM frequency: ~19500 Hz
  #else
  analogWriteFrequency(19500);             //set PWM frequency to 19.5kHz to reduce motor noise
  analogWriteResolution(8);                //set ADC resolution to 8 bits
  #endif
  for (int i=0; i < MOTOR_MAX; i++){
    pinMode(Pin1[i], OUTPUT);              //define ADC pins as outputs
    pinMode(Pin2[i], OUTPUT);              //define ADC pins as outputs
  }
  for (int i=0; i < MOTOR_MAX; i++){
    FUN_doStop(i);                         //execute stop command for all motors
  }
  Serial.println("init Motor driver done");
}
//*****************************************************************************************************************
// arduino IDE standard calls
void setup() {
  Serial.begin(115200);                    //enable serial interface with 115200 baud
  initMotorDriver();                       //init Motor driver
  initWifi();                              //init Wifi and OTA (after one time flashing via USB PORT, next time the ESP32 can be flashed via WLAN
}
// arduino IDE main loop procedure
void loop() {
  ArduinoOTA.handle();                     //needed to act on OTA requests
  loop_Watchdog();                         //cyclic execute watchdog
  loop_AkkuVoltageRead();                  //cyclic read voltage
}

// helper function to convert IP address in string
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]);
}
