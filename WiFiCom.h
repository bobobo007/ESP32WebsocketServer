/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version V06.010
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   Library for WiFiCom.h
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
#ifndef WiFiCom
#define WiFiCom

#include "V06_003.h"
#include <WiFi.h>             //Library for WiFi
#include <WiFiClient.h>       //Library for WiFi Client
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

class WiFiComm
{
  public:
  /**
   * @brief Function to WiFi setup
   * @param noting
   * @retval true if WiFi is connected false if not
   */
  bool WiFiSetup(void);
  /**
   * @brief Mein WiFi Function
   * @param noting
   * @retval true if WiFi is connected false if not
   */
  bool WiFimain(void);
  /**
   * @brief Function to handle WebSocket Message
   * @param 
   * @param Message data
   * @param Lenght of the data
   * @param Websocketclient
   * @retval true if WiFi is connected false if not
   */
  void handleWebSocketMessage(void *arg, uint8_t *data, size_t len, AsyncWebSocketClient *client);
  /**
   * @brief Function to  send initial data
   * @param Websocketclient
   * @retval true if WiFi is connected false if not
   */
  void sendInitialData(AsyncWebSocketClient *client);
  /**
   * @brief Function to check and send all server data 
   * @param noting
   * @retval noting
   */
  void checkAndSendChanges(void);
  /**
   * @brief Function to  send Json update Data
   * @param key
   * @param value
   * @retval nothing
   */
  void sendJsonUpdate(const char* key, DynamicJsonDocument& value);
  /**
   * @brief Function to  send log fail
   * @param nothing
   * @retval nothing
   */
  void sendLogFile(void);
  /**
   * @brief Function to  send error message
   * @param error message
   * @retval nothing
   */
  void sendErrorMessage(const char *errormessage);
  /**
   * @brief Function to  send WebPage
   * @param nothing
   * @retval nothing
   */
  bool sendWebPage();
};
#endif /* WiFiCom */
