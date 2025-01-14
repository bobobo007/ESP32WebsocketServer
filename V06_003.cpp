/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version V06.003
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   V006_003 hardware
 *
\verbatim
 * |----------------------------------------------------------------------
 * | Copyright (c) 2024 Bohumir Coufal
 * |
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software,
 * | and to permit persons to whom the Software is furnished to do so,
 * | subject to the following conditions:
 * |
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * |
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */

//Incklude Hardware V06_003
#include "V06_003.h"

//DS18B20 libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Libraries for MCP79410 RTC connected via I2C and Wire lib
#include "Wire.h"
#include "RTClib.h"

// Libraries to get time from NTP Server
#include <time.h>
#include <NTPClient.h>
#include "Functions.h"
#include "WiFiCom.h"

//RTC Typ
RTC_MCP79410 rtc;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
String inputsStamps;
String logmes;

// Counter for Inputs
bool inputState[]={false, false, false, false};
bool outputState[]={false, false, false, false};
bool flagAlarm = false;
uint16_t counti = 0;
uint8_t counti1 = 0;
uint8_t counti2 = 0;
uint8_t counti3 = 0;
uint8_t counti4 = 0;
uint8_t logcount = 0;
unsigned long epocht;
bool lastOutput[4] = {"false","false","false","false"};
// Objects
Hardware hdw;
WiFiComm wicom;
Cleaning cisti;

// Setup inputs and outputs
void Hardware::funkSetupHardware(void) {
  pinMode(DRAIN_VENTIL, OUTPUT);
  pinMode(CLEANING_VENTIL, OUTPUT);
  pinMode(PREHEATING, OUTPUT);
  pinMode(OUTPUT4, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(DRAIN_VENTIL, HIGH);
  digitalWrite(CLEANING_VENTIL, HIGH);
  digitalWrite(PREHEATING, HIGH);
  digitalWrite(OUTPUT4, HIGH);
  digitalWrite(RED_LED, HIGH);
  pinMode(DRAIN_POSITION, INPUT);
  pinMode(CLEANING_POSITION, INPUT);
  pinMode(TANKFULL, INPUT);
  pinMode(INPUT4, INPUT);
  pinMode(DCOFF, INPUT);
  pinMode(INT, INPUT);
}

// Function for defining drain times
void Hardware::funkDefineDrainTime(void) {
  drainTime[0].hour = 8;
  drainTime[0].minute = 0;
  drainTime[0].second = 0;
  drainTime[1].hour = 16;
  drainTime[1].minute = 0;
  drainTime[1].second = 0;
  drainTime[2].hour = 0;
  drainTime[2].minute = 0;
  drainTime[2].second = 0;
}

// Initialize an NTPClient to get time
void Hardware::funkSetupNTP(void) {
  timeClient.begin();
  timeClient.setTimeOffset(3600); // Sets offset to 3600 seconds at startup
}

// Function to retrieve time from an NTP server
void Hardware::funkGetNTPTime(DateTimeTypeDef *ntime) {
  // Update time from NTP
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  Serial.println("NTP time downloaded!");
  // Retrieve epoch time
  time_t epocht = timeClient.getEpochTime();
  // Parse epoch time into a tm structure
  struct tm *timeInfo = gmtime(&epocht);
  if (timeInfo != nullptr) {
    // Set the values in DateTimeTypeDef
    ntime->year = timeInfo->tm_year + 1900; // Years since 1900
    ntime->month = timeInfo->tm_mon + 1;    // Months from 0 to 11
    ntime->day = timeInfo->tm_mday;         // Day of the month
    ntime->hour = timeInfo->tm_hour;        // Hour
    ntime->minute = timeInfo->tm_min;       // Minute
    ntime->second = timeInfo->tm_sec;       // Second
    // Check if daylight saving time (DST) is active
    bool dst = hdw.funkisDST();
    int offset = dst ? 7200 : 3600; // 7200 for DST, 3600 for standard time
    timeClient.setTimeOffset(offset);
    Serial.printf(
      "NTP Time: %04d-%02d-%02d %02d:%02d:%02d (DST: %s, Offset: %d sec)\n",
      ntime->year, ntime->month, ntime->day,
      ntime->hour, ntime->minute, ntime->second,
      dst ? "YES" : "NO", offset
    );
    logmes = "NTPSet";      
  } else {
    // Error retrieving time
    Serial.println("Failed to parse NTP time");
    logmes = "NTPError"; 
    wicom.sendErrorMessage(logmes.c_str());
  }
  hdw.funkLogSDCard(logmes.c_str());
}


bool Hardware::funkisDST(void) {
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  // If we're not in March or October, test simple conditions
  if (month < 3 || month > 10) {
    return false; // Standard time
  }
  if (month > 3 && month < 10) {
    return true; // Daylight Saving Time
  }
  // For March (DST starts on the last Sunday)
  if (month == 3) {
    // Find the last Sunday of March
    int lastSunday = 31 - (DateTime(year, 3, 31).dayOfTheWeek());
    if (day < lastSunday || (day == lastSunday && hour < 2)) {
      return false; // Still standard time
    } else {
      return true; // Daylight Saving Time
    }
  }
  // For October (DST ends on the last Sunday)
  if (month == 10) {
    // Find the last Sunday of October
    int lastSunday = 31 - (DateTime(year, 10, 31).dayOfTheWeek());
    if (day < lastSunday || (day == lastSunday && hour < 3)) {
      return true; // Still Daylight Saving Time
    } else {
      return false; // Standard time
    }
  }
  return false; // If we get here, something went wrong
}


//Function to start temperature measurement
void Hardware::funkStartTemperatureReading(void){
  sensors.begin();
}

//Function to get temperature
float Hardware::funkGetTemperature(void){
  float temperature;
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0); // Temperature in Celsius
  //temperature = sensors.getTempFByIndex(0); // Temperature in Fahrenheit
  return temperature;
}

void Hardware::funkStartRTC(void)
{
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);  // don't go anywhere!
  }
    // Check to see if RTC is running. If not, set the date to settings below. If it is, assume
    // that it is set correctly, and doesn't need to be adjusted. REMOVE THIS CHECK IF YOU WANT
    // TO OVERRIDE THE CURRENT RTC SETTINGS!
    if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting date/time!");
    // following line sets the RTC to the date & time this sketch was compiled
    //                    Y   M  D   H   M   S    enable battery backup
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)), MCP7941X_BATT_BKUP_EN);
    // To set the RTC with an explicit date & time, see below
    // For May 31, 2016 at 1:23:16 :
    //                      Y   M  D   H   M   S    enable battery backup
    //rtc.adjust(DateTime(2016, 5, 31, 1, 23, 16), MCP7941X_BATT_BKUP_EN);
  } else {
    Serial.println("RTC already running... no time adjust");
  }
}

void Hardware::funkSetRTCTime(void) {
  DateTimeTypeDef timentp[1];
  hdw.funkGetNTPTime(timentp);
  rtc.adjust(DateTime(timentp[0].year, timentp[0].month, timentp[0].day, timentp[0].hour, timentp[0].minute, timentp[0].second), MCP7941X_BATT_BKUP_EN);
  Serial.println("RTC has bin ajusted");
}

uint8_t Hardware::funkSetupSDCard(void) {   
  // initialize SD card
  if(!SD.begin(SD_CS)){
    Serial.println("Card Mount Failed");
    return 0;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return 0;
  }
  // initialize SD card
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return 0;    // init failed
  }
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    file.close();
    return 1;
  }
  else {
    Serial.println("File already exists");  
    return 2;
  }
}

void Hardware::funkDeleteLogFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  boolean a = fs.remove(path);
  if(!a) {
    Serial.println("Failed to delete file");
    logmes = "FailedToDelete" ;
  }
  else {
    Serial.println("File deleted");
    logmes = "LogFileDeleted" ;
  }
  funkSetupSDCard();
  logCounter = 1;
  rtc.funkWriteSRAM();
  hdw.funkLogSDCard(logmes.c_str());
}

int Hardware::funkReadInputs(void) {
  int res = 0;
  // Save the current states of inputs
  memcpy(inputState, input, sizeof(input));
  // Increment input counters
  counti++;
  counti1 += digitalRead(DRAIN_POSITION);
  counti2 += digitalRead(CLEANING_POSITION);
  counti3 += digitalRead(TANKFULL);
  counti4 += digitalRead(INPUT4);
  // Check every 10 cycles
  if (counti == 10) {
    input[0] = counti1 > 3;
    input[1] = counti2 > 3;
    input[2] = counti3 > 3;
    input[3] = counti4 > 3;
    // Reset counters
    counti = counti1 = counti2 = counti3 = counti4 = 0;
  }
  // Detect changes in inputs
  for (int i = 0; i < 4; ++i) {
    if (inputState[i] != input[i]) {
      res = i + 1;
      break;
    }
  }
  return res;
}

int Hardware::funkReadOutputs(void) {
  int outputIndex = 0;
  output[0] = digitalRead(DRAIN_VENTIL);
  output[1] = digitalRead(CLEANING_VENTIL);
  output[2] = digitalRead(PREHEATING);
  output[3] = digitalRead(OUTPUT4);
  for (int i = 0; i < 4; i++) {
    if (lastOutput[i] != output[i]) {
      lastOutput[i] = output[i];
      Serial.println("outputsChanged");
      outputIndex = i+1;
    }
  }
  return outputIndex;
}

void Hardware::funkReadAll(void) {
  DateTimeTypeDef rtct[0];
  // Start the DallasTemperature library
  hdw.funkStartTemperatureReading();
   // Read the temperature
  valveTemperature = hdw.funkGetTemperature();
  // Read the distanc
  waterDepth = 1250;
  // Read RTC time
  DateTime now = rtc.now();
  currentTime.year = now.year();
  currentTime.month = now.month();
  currentTime.day = now.day();
  currentTime.hour = now.hour();
  currentTime.minute = now.minute();
  currentTime.second = now.second();
}

void Hardware::funkLogSDCard(const char *logmessage, DateTimeTypeDef time) {
  char messageBuffer[256]; // Buffer for one message (large enough for all data)
  snprintf(
    messageBuffer, sizeof(messageBuffer),
    "%u,%04u-%02u-%02u,%02u:%02u:%02u,%.2f,%d,%d%d%d%d,%d%d%d%d,%s\r\n",
    logCounter,
    time.year, time.month, time.day,
    time.hour, time.minute, time.second,
    valveTemperature,
    waterDepth,
    input[0], input[1], input[2], input[3],
    !(digitalRead(DRAIN_VENTIL)), !(digitalRead(CLEANING_VENTIL)), !(digitalRead(PREHEATING)), !(digitalRead(OUTPUT4)),
    logmessage
  );
  Serial.print("Save data: ");
  Serial.print(messageBuffer);
  // Write to SD card
  funkAppendFile(SD, "/data.txt", messageBuffer);
  // Counter incrementation
  logCounter = (logCounter < COUNTERLOG) ? logCounter + 1 : 1;
  rtc.funkWriteSRAM();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void Hardware::funkWriteFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void Hardware::funkAppendFile(fs::FS &fs, const char * path, const char * message) {
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void Hardware::funkToggleLED(void) {
  if(digitalRead(RED_LED)) {
    digitalWrite(RED_LED, LOW);
  }
  else {
    digitalWrite(RED_LED, HIGH);
  }
}

void Hardware::funkSendInputsLogs(int in) {
  switch (in) {
    case 1: 
      logmes = "drainInput"; 
      logmes += input[0] ? "On" : "Off";
      break;
    case 2: 
      logmes = "cleanInput"; 
      logmes += input[1] ? "On" : "Off";
      break;
    case 3: 
      logmes = "tankInput"; 
      logmes += input[2] ? "On" : "Off";
      break;
    case 4: 
      logmes = "input4"; 
      logmes += input[3] ? "On" : "Off";
      break;
    default:
      logmes = "invalideInputIndex";
    break;
  }
  hdw.funkLogSDCard(logmes.c_str());
}

void Hardware::funkSendOutputsLogs(int out) {
  switch (out) {
    case 1: 
      logmes = "drainVentil"; 
      logmes += !digitalRead(DRAIN_VENTIL) ? "On" : "Off";
      break;
    case 2: 
      logmes = "cleanVentil"; 
      logmes += !digitalRead(CLEANING_VENTIL) ? "On" : "Off";
      break;
    case 3: 
      logmes = "preheat"; 
      logmes += !digitalRead(PREHEATING) ? "On" : "Off";
      break;
    case 4: 
      logmes = "output4"; 
      logmes += !digitalRead(OUTPUT4) ? "On" : "Off";
      break;
    default:
      logmes = "invalideOutputIndex";
    break;
  }
  hdw.funkLogSDCard(logmes.c_str());
}
