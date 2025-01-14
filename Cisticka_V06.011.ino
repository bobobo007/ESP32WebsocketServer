/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version V06.010
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   Cisticka_V06.010.ino
 *
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2024 Bohumir Coufal

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
*/
  
// Load required libraries
#include "esp_system.h"       //Library for WatchdogTimer

//Library for Hardware
#include "V06_003.h"
#include "Functions.h"
#include "WiFiCom.h"
#include "RTClib.h"

//Objects
Hardware hard;
Cleaning clea;
WiFiComm WiC;
RTC_MCP79410 rtc1;

//Defines variables
String logm;                     //Message for Logfile
bool WifiConnected = false;      //WiFiConected ok or not
int WiFiCounter = 0;             //WiFiCounter are used for restart WiFi after WiFiRESTART defined in V06.003.h
int sdKardOk = 0;                //SD Kart inicialisation OK
DateTimeTypeDef stamp[0];

//Extern variables
bool cleaningOn;                 //Cleaning On
bool drainOn;                    //Drain Valve On
float valveTemperature;          //Temperature on drain valve
int waterDepth;                  //Wtaer depth
bool input[4];                   //Inputs
bool output[4];                  //Outputs
DateTimeTypeDef currentTime;     //Current time
int logCounter;                  //Log counter
TimeTypeDef drainTime[3];        //drain time
bool flagDrainOn;                //Flag when Cleaning has bin started

unsigned long startTimer1, startTimer2;         //Timer1

const int wdtTimeout = 5000;         //time in ms to trigger the watchdog
hw_timer_t *watchdogTimer = NULL;

void IRAM_ATTR resetModule() {      //Interrupt from WatchdogTimer
  Serial.println("Reboot from Wachdog");
  esp_restart();
}

void setup(){
  // Start serial communication for debugging purposes
  Serial.begin(115200); 
  // Initialize and start RTC chip
  hard.funkStartRTC();
  //Define odkal times
  hard.funkDefineDrainTime();
  // Read values from RTC SRAM
  rtc1.funkReadSRAM();  
  // Initialise Inputs and outputs
  hard.funkSetupHardware();
  // Initialize SD Card;
  sdKardOk = hard.funkSetupSDCard();     
  if (sdKardOk == 1) {    //Whenn new file has been created delete the counter
    logCounter = 1;
    rtc1.funkWriteSRAM();
  }
  if (sdKardOk == 0) {
    hard.funkLogSDCard("SDCardError");
  }
  // Start reading all values
  hard.funkReadAll();
  // Initialise Timer1
  startTimer1 = millis();
  // Initialise WiFi
  WifiConnected = WiC.WiFiSetup();
  if (WifiConnected) {
    hard.funkSetupNTP();
  }
  else {
    logm = "WiFiNotConnected";
    hard.funkLogSDCard(logm.c_str());
  }
  //Print RTC Register
  //rtc1.funkPrintRTCRegister();
  //If the power supply voltage has been switched off, read the time stamp and write it to the log file
  rtc1.funkReadTimeStamp(stamp);
  if(rtc1.VccFault()) {
    logm = "VCCDisconnected";
    hard.funkLogSDCard(logm.c_str(), stamp[0]);
    logCounter++;
  }
  //Showing software and hardware versions   
  logm = "Start device: Purifier: H";
  logm += HARDWARE;
  logm += " S";
  logm += SOFTWARE;
  Serial.println(logm);
  logm = "Start";
  hard.funkLogSDCard(logm.c_str());
  watchdogTimer = timerBegin(1000);                      //timer 1khz resolution
  timerAttachInterrupt(watchdogTimer, &resetModule);     //attach callback
  timerAlarm(watchdogTimer, wdtTimeout * 1, false, 0);   //set time in ms
}
  
void loop(){
  timerWrite(watchdogTimer, 0);            //reset timer (feed watchdog)
  long loopTime = millis();
  if (millis() - startTimer1 >= 200) {     //Timer1 200ms
    // Start reading all values
    hard.funkReadAll();
    WiFiCounter++;
    hard.funkToggleLED();
    startTimer1 = millis();                //Timer1 end
   }
   if (millis() - startTimer2 >= 12) {     //Timer2 12ms
    // Read Inputs
    int inputIndex = hard.funkReadInputs();
    // If any input has been changed write log
    if (inputIndex > 0) hard.funkSendInputsLogs(inputIndex);
    int outputIndex = hard.funkReadOutputs();
    if (outputIndex > 0) hard.funkSendOutputsLogs(outputIndex); 
    startTimer2 = millis();                //Timer2 end
   }
  if((WiFiCounter >= WiFiRESTART)&&(!WifiConnected)) {
    WifiConnected = WiC.WiFiSetup();
    if (!WifiConnected) {
      logm = "WiFicould";
      hard.funkLogSDCard(logm.c_str());
    }
    WiFiCounter = 0;
  }
  // Function for cleaning ventil
  clea.funkCleaningVentil();
  // Function for drain valve
  clea.funDrainVentil();
  if (WifiConnected) {
    WifiConnected = WiC.WiFimain();    //if WiFi still connected WifiConnected = 1
  }
  loopTime = millis() - loopTime;       //WatchdogTimer
}
