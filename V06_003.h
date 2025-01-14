/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version V06.003
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   Library for V006_003 hardware
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

#ifndef V06_003
#define V06_003

#if defined (ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
    #if defined (PARTICLE)
      #include <SparkIntervalTimer.h>
    #else
      #include <pins_arduino.h>
    #endif
  #endif

  #if defined (__AVR__)
    #include <avr/io.h>
    #include <avr/interrupt.h>
  #endif

// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

#define SOFTWARE "V06.011"
#define HARDWARE "V06.003"
// Define CS pin for the SD card module
#define SD_CS 5

//Define Outputs 1 - 4 refers to ESP32 GPIO 25, 26, 27, 4
#define DRAIN_VENTIL 25
#define CLEANING_VENTIL 26
#define PREHEATING 27
#define OUTPUT4 14
#define RED_LED 2
//Define Inputs 1 - 4 refers to ESP32 GPIO 34, 35, 32, 33
#define DRAIN_POSITION 34
#define CLEANING_POSITION 35
#define TANKFULL 32
#define INPUT4 33
#define DCOFF 13
#define INT 4

//Data wire is connected to ESP32 GPIO 15
#define ONE_WIRE_BUS 15

#define WiFiRESTART 60        //After this value WiFi will restart value is in milisecond
#define NTP_TIMEOUT 1500      //Timeout for NTP server in miliseconds
#define COUNTERLOG  10000     //Maximum logs on SD card

#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ) )

struct TimeTypeDef{  /* time structur */
  uint8_t hour;       
  uint8_t minute;     
  uint8_t second;    
};

struct DateTimeTypeDef{ /* date structur */
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;       
  uint8_t minute;     
  uint8_t second;     
};

extern bool cleaningOn;                 //Switching cleaning/recirculation
extern bool drainOn;                    //Drain Valve On
extern float valveTemperature;          //Valve temperature
extern int waterDepth;                  //Depth of water in the cleaner
extern bool input[4];                   //Inputs
extern bool output[4];                  //Outputs
extern DateTimeTypeDef currentTime;     //current time
extern int logCounter;                  //log entry counter
extern TimeTypeDef drainTime[3];        //unlock time
extern bool flagDrainOn;                //Flag when draing has bin started

class Hardware
{
  public:

  /**
   * @brief Function for setup Inputs and Outputs
   * @retval  nothing
   */
  void funkSetupHardware(void);
  /**
   * @brief Function for define drain times
   * @retval  nothing
   */
  void funkDefineDrainTime(void);
  /**
   * @brief Function for setup NTPClient to get time
   * @param nothing
   * @retval  nothing
   */
  void funkSetupNTP(void);
  /**
   * @brief Function to detect whether it is wintertime or summertime
   * @param NTP time pointer
   * @retval  nothing
   */
  void funkGetNTPTime(DateTimeTypeDef *ntime);
  /**
   * @brief Function to detect whether it is wintertime or summertime
   * @param date time pointer
   * @retval  true - summertime, false - wintertime
   */
  bool funkisDST(void);
  /**
   * @brief Funktion to start temperature measurement
   * @param nothing
   * @retval  nothing
   */
  void funkStartTemperatureReading(void);
  /**
   * @brief Function for read temerature
   * @param nothing
   * @retval  temperature
   */
  float funkGetTemperature(void);
  /**
   * @brief Function search for the string sfind in the string str
   * @param for the string sfind in the string str
   * @retval  1 if string found, 0 if string not found
   */
  void funkStartRTC(void);
  /**
   * @brief Function for set RTC Time and Date in RTC chip
   * @param nothing
   * @retval  notimg
   */
  void funkSetRTCTime(void);
  /**
   * @brief Function for set the SD Card
   * @param nothing
   * @retval return 1 when new file has been created otherwise 0
   */
  uint8_t funkSetupSDCard(void);
  /**
   * @brief Function for delete Logfile on the SD Card
   * @param path
   * @retval nothing
   */
  void funkDeleteLogFile(fs::FS &fs, const char * path);
  /**
   * @brief Function for read the Inputs
   * @param nothing
   * @retval return number of inputs wat has been changed
   */
  int funkReadInputs(void);
  /**
   * @brief Function for read the Outputs
   * @param nothing
   * @retval return number of outputs wat has been changed
   */
  int funkReadOutputs(void);
  /**
  * @brief Function for write read teperature, distanc, RTC time and date
  * @param nothing
  * @retval nothing
  */
  void funkReadAll(void);
  /**
   * @brief Function for write logmessage to SD card
   * @param time
   * @param logmessage
   * @retval nothing
   */
  void funkLogSDCard(const char *logmessage = "Default message", DateTimeTypeDef time = currentTime);
   /**
   * @brief Function for write file to SD card
   * @param path
   * @param message
   * @retval nothing
   */
  void funkWriteFile(fs::FS &fs, const char * path, const char * message);
  /**
   * @brief Function for append file to SD card
   * @param path
   * @param message
   * @retval nothing
   */
  void funkAppendFile(fs::FS &fs, const char * path, const char * message);
  /**
   * @brief function to toggle the LED
   * @param nothing
   * @retval nothing
   */
  void funkToggleLED(void);
  /**
   * @brief function to send Inputs Logs
   * @param nummer of the input
   * @retval nothing
   */
  void funkSendInputsLogs(int input);
  /**
   * @brief function to send Outputs Logs
   * @param nummer of the output
   * @retval nothing
   */
  void funkSendOutputsLogs(int out);

};

#endif /* V06_003 */
