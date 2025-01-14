/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version v1.0
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   Functions.cpp
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

//Incklude Hardware V06.003
#include "V06_003.h"
#include "Functions.h"
#include "RTClib.h"
#include "WiFiCom.h"

// Variable definitions in the programme
//*******************************************
// An array that stores information about whether the temperature was lower than MINTEMPERATURE at the given hour.
// Index 0 = hour 0:00-0:59, 1 = hour 1:00-1:59, ... 23 = hour 23:00-23:59.
bool hourlyBelowMin[24] = {true};
bool flagTemperatureStatus = false;
String logme;                    // Message for Logfile

//object
Hardware hardw;
RTC_MCP79410 rtca;
WiFiComm wcom;


void Cleaning::funDrainVentil(void) {
  static unsigned long drainValveStartTime = 0;
  static bool isDrainValveActive = false;
  static unsigned long preheatStartTime = 0;
  static bool isPreheatActive = false;
  static bool criticalTempErrorReported = false;
  static bool preheatTimeoutErrorReported = false;
  static bool taskInProgress = false; // Added status flag for ongoing task

  bool temperatureBelowMin = funTemperatureStatus();
  // Check if the current time matches one of the times in drainTime
  bool timeMatch = false;
  for (int i = 0; i < 3; i++) {
    if (currentTime.hour == drainTime[i].hour &&
        currentTime.minute == drainTime[i].minute &&
        currentTime.second == drainTime[i].second) {
      timeMatch = true;
      break;
    }
  }
  if (timeMatch || drainOn || taskInProgress) { // If time matches, a command is active, or a task is in progress
    if (!taskInProgress) { // If the task hasn't started yet, initialize it
      taskInProgress = true;
      drainOn = false;
      flagDrainOn = true;
    }
    if (temperatureBelowMin) {
      // Reset error flags
      criticalTempErrorReported = false;
      preheatTimeoutErrorReported = false;
      // Activate the DrainValve to LOW for DRAIN_VALVE_TIME
      digitalWrite(DRAIN_VENTIL, LOW);
      drainValveStartTime = millis();
      isDrainValveActive = true;
      taskInProgress = false; // Task completed
    } else if (valveTemperature > ABSOLUT_TEMPERATURE) {
      // If the temperature is above the critical threshold, activate Preheat
      if (!isPreheatActive) {
        digitalWrite(PREHEATING, LOW);
        preheatStartTime = millis();
        isPreheatActive = true;
      }
      if (valveTemperature >= PREHEAT_TEMPERATURE) {
        // Deactivate Preheat and activate DrainValve
        digitalWrite(PREHEATING, HIGH);
        digitalWrite(DRAIN_VENTIL, LOW);
        drainValveStartTime = millis();
        isDrainValveActive = true;
        isPreheatActive = false;
        taskInProgress = false; // Task completed
      } else if (millis() - preheatStartTime > MAX_PREHEAT_TIME) {
        // If the desired temperature is not reached after an hour
        if (!preheatTimeoutErrorReported) {
          logme = "temperatureNotReached";
          hardw.funkLogSDCard(logme.c_str());
          wcom.sendErrorMessage(logme.c_str());
          Serial.println("The desired temperature at the valve was not reached");
          preheatTimeoutErrorReported = true;
        }
        digitalWrite(PREHEATING, HIGH);
        isPreheatActive = false;
        taskInProgress = false; // Task completed
        flagDrainOn = false;
      }
    } else {
      // If the temperature is below the critical threshold
      if (!criticalTempErrorReported) {
        logme = "temperatureTooLow";
        hardw.funkLogSDCard(logme.c_str());
        wcom.sendErrorMessage(logme.c_str());
        Serial.println("The temperature at the valve is below the critical threshold");
        criticalTempErrorReported = true;
      }
      taskInProgress = false; // Task completed
      flagDrainOn = false;
    }
  }
  // Deactivate DrainValve after DRAIN_VALVE_TIME
  if (isDrainValveActive && millis() - drainValveStartTime >= DRAIN_VALVE_TIME) {
    digitalWrite(DRAIN_VENTIL, HIGH);
    isDrainValveActive = false;
    flagDrainOn = false;
  }
}


//Function to control the three-way valve
//*******************************************
void Cleaning::funkCleaningVentil(void) {
  if ((cleaningOn) && !(input[2])) {
    digitalWrite(CLEANING_VENTIL, LOW);          //if it is to be cleaned and the valve is not in the cleaning position, switch on cleaning
	}
	else {
    digitalWrite(CLEANING_VENTIL, HIGH);
  }
}

bool Cleaning::funTemperatureStatus(void) {
  // For example, call this code every minute or second.
  // If we only want to write the result at the beginning of each hour, we can check, 
  // when minute and second is 0, i.e. when is exactly the new hour.
  DateTime now = rtca.now();
  if ((currentTime.minute == 0) && (currentTime.second == 0)) {
    if (!flagTemperatureStatus) {
      flagTemperatureStatus = true;
      // If the temperature at the beginning of the new hour was lower than MINTEMPERATURE, we note this.
      if (valveTemperature < MINTEMPERATURE) {
        hourlyBelowMin[currentTime.hour] = true;
      } else {
        // At this hour the temperature has not dropped below MINTEMPERATURE.
        hourlyBelowMin[currentTime.hour] = false;
      }
    }
  } else {
    flagTemperatureStatus = false;
  }
  // Now we check if in the last 24 hours (i.e. in all indexes) 
  // the temperature has been below MINTEMPERATURE at least once.
  for (int i = 0; i < 24; i++) {
    if (hourlyBelowMin[i] == true) {
      // In at least one hour in the last 24 hours the temperature dropped below MINTEMPERATURE
      return false;
    }
  }
  // If we did not find any hour where the temperature was below MINTEMPERATURE, return true.
  return true;
}
