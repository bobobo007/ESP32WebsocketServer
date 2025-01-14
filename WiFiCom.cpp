/**
 * @author  Bohumir Coufal
 * @email   bohumir.coufal@arcor.de
 * @version V06.010
 * @ide     Arduino IDE 2.3.4
 * @license MIT
 * @brief   WiFiComm.cpp
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

//WebSocked Commands
//com   Command
//time  Time
//sta   State
//gv    Get Values
//so    Software Version
//ha    Hardware Version
//te    Temperature at the valve
//de    Depth of water in the tank
//wi    WiFi RSSI value in dB
//cl    Cleaning or Recirkulacion
//su    Summertime or Wintertime
//ip    Inputs
//ou    Outputs
//dr    Open drain valve
//st    Set time from celular
//nt    Set time from ntp server
//fl    Flag when Cleaning has bin started
//dl    Delete log file
//sl    Send Logfile
//er    Error Message
//lf    LogFile

// Load required libraries
#include <ArduinoJson.h>
#include "WiFiCom.h"
#include <WiFi.h>             //Library for WiFi
#include <WiFiClient.h>       //Library for WiFi Client
#include "SPIFFS.h"
#include "V06_003.h"
#include "RTClib.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// defines variables
#define REQ_BUF_SZ   128          //Buffer size

String info;
String logmess;                   //Message for Logfile
float lastTemperature = 0;
int lastDepth, lastWiFirssi;
bool lastClean = false;
bool lastSummer = false;
bool lastflagDrainOn = false;
bool lastInput[4];
bool lOutput[4];
bool flagServer = false;
bool connected = false;

Hardware harw;
WiFiComm WiCo;
RTC_MCP79410 rtcw;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//Your Base64 encoded string (name:password)
const char* expectedAuth = "YWRtaW46YWRtaW4="; // Base64 for "admin:admin"

//Replace with your network credentials
const char* ssid     = "XYNetwork";
const char* password = "123456789";

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 100);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 1, 254);     //optional
IPAddress secondaryDNS(192, 168, 1, 254);   //optional


bool WiFiComm::WiFiSetup(void) {
  int conn = 0;     //Trial counter
  int result = 0;
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while ((WiFi.status() != WL_CONNECTED)&&(conn < 15)) {
    delay(500);
    Serial.print(".");
    conn++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.printf("%4d", WiFi.RSSI());
    Serial.println("dB");
    WiCo.sendWebPage();

    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    //WebSocket Initialization
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.begin();
    return true;
  } else {
    return false;
  }
}

bool WiFiComm::WiFimain(void) {
  ws.cleanupClients(); // Cleaning inactive WebSocket clients
  if(connected) WiCo.checkAndSendChanges();
  if(WiFi.status() == WL_CONNECTED) {
    return true;
  } else {
    WiFi.disconnect();
    delay(1);
    logmess = "WiFiDisconnected";
    harw.funkLogSDCard(logmess.c_str());
    Serial.println(logmess);
    return false;
  }
}

void WiFiComm::handleWebSocketMessage(void *arg, uint8_t *data, size_t len, AsyncWebSocketClient *client) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->opcode != WS_TEXT) return;
  data[len] = '\0';
  Serial.printf("Received WebSocket message: %s\n", (char *)data);
  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, (char *)data)) {
    Serial.println("Failed to parse WebSocket message");
    return;
  }
  const char *command = doc["com"] | "";
  if (strlen(command) == 0) {
    Serial.println("Key 'com' not found in the JSON message");
    return;
  }
  auto handleStateCommand = [&](const char *key, auto action) {
    if (!doc.containsKey(key)) {
      Serial.printf("Key '%s' not found in the JSON message\n", key);
      return;
    }
    action(doc[key]);
  };
  if (strcmp(command, "dr") == 0) {
    handleStateCommand("sta", [&](bool state) {
      logmess = "DrainOn";
      harw.funkLogSDCard(logmess.c_str());
      drainOn = true;
    });

  } else if (strcmp(command, "cl") == 0) {
    handleStateCommand("sta", [&](bool state) {
      cleaningOn = state;
      logmess = state ? "SwitchCleaning" : "SwitchRecirkulacion";
      harw.funkLogSDCard(logmess.c_str());
    });

  } else if (strcmp(command, "gv") == 0) {
    handleStateCommand("sta", [&](bool state) {
    WiCo.sendInitialData(client);
    Serial.println("Initialisation data is sent");
    });
  } else if (strcmp(command, "sl") == 0) {
    handleStateCommand("sta", [&](bool state) {
      cleaningOn = state;
      WiCo.sendLogFile();
    });
  } else if (strcmp(command, "dl") == 0) {
    handleStateCommand("sta", [&](bool state) {
      Serial.println("Logfile is deleted");
      harw.funkDeleteLogFile(SD, "/data.txt");
    });
  } else if (strcmp(command, "st") == 0) {
    int year, month, day, hour, minute, second;
    handleStateCommand("time", [&](const char *timestamp) {
      if (sscanf(timestamp, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) == 6) {
        rtcw.adjust(DateTime(year, month, day, hour, minute, second), MCP7941X_BATT_BKUP_EN);
        harw.funkLogSDCard("TimeSet");
        Serial.printf("A new time has been set: %s\n", timestamp);
      } else {
        Serial.println("Failed to parse timestamp.");
        harw.funkLogSDCard("TimeSetError");
        WiCo.sendErrorMessage("TimeSetError");
      }
    });
  } else if (strcmp(command, "nt") == 0) {
    handleStateCommand("sta", [&](bool state) {
      Serial.println("Setting up time from NTP");
      harw.funkSetRTCTime();
    });

  } else if (strncmp(command, "ou", 2) == 0) {
    handleStateCommand("sta", [&](bool state) {
      int outputIndex = atoi(command + 2);
      int pin;
      switch (outputIndex) {
        case 1: pin = DRAIN_VENTIL; break;
        case 2: pin = CLEANING_VENTIL; break;
        case 3: pin = PREHEATING; break;
        case 4: pin = OUTPUT4; break;
        default:
          Serial.println("Invalid output index");
        return;
      }
      char messageBuffer[32]; // Buffer for one message (large enough for all data)
      snprintf(
        messageBuffer, sizeof(messageBuffer),
        "Output %d SwitchedTo: %s\n", outputIndex, state ? "ON" : "OFF"
      );
      Serial.print(messageBuffer);  
      digitalWrite(pin, state ? LOW : HIGH);
    });
  } else {
    Serial.println("Unknown command");
  }
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      IPAddress clientIP = client->remoteIP();
      Serial.printf("WebSocket client connected: %u, IP: %s\n", client->id(), clientIP.toString().c_str());  
      //Client with IP adress for Log file
      String logMessage = "clientConnected, ID: " + String(client->id()) + ", IP: " + clientIP.toString();
      harw.funkLogSDCard(logMessage.c_str());  
      connected = true;
      break;
    }
    case WS_EVT_DISCONNECT: {
      Serial.printf("WebSocket client disconnected: %u\n", client->id());
      connected = false;
      break;
    }
    case WS_EVT_DATA: {
      WiCo.handleWebSocketMessage(arg, data, len, client);
      break;
    default:
      Serial.printf("Unhandled WebSocket event: %d\n", type);
      break;
    }
  }
}



void WiFiComm::sendInitialData(AsyncWebSocketClient *client) {
  DynamicJsonDocument doc(128);
  doc["so"] = SOFTWARE;
  doc["ha"] = HARDWARE;
  doc["te"] = valveTemperature;
  doc["de"] = 1205; //or other method to obtain the depth of the liquid
  doc["wi"] = WiFi.RSSI();
  doc["cl"] = cleaningOn;
  doc["fl"] = flagDrainOn;
  JsonArray inputs = doc.createNestedArray("ip");
  for (int i = 0; i < 4; i++) {
    inputs.add(input[i]);
  }
  JsonArray outputs = doc.createNestedArray("ou");
  for (int i = 0; i < 4; i++) {
    outputs.add(!(output[i]));
  }
  String jsonString;
  serializeJson(doc, jsonString);
  client->text(jsonString);
  Serial.print("I sent the data via Websocet: ");
  Serial.println(jsonString);
}

void WiFiComm::checkAndSendChanges(void) {
  DynamicJsonDocument doc(32);
  doc.clear();
  // Temperature update
  if (abs(valveTemperature - lastTemperature) > 0.5) {
    lastTemperature = valveTemperature;
    doc["te"] = valveTemperature;
    sendJsonUpdate("te", doc);
  }
  // Depth update
  if (abs(waterDepth - lastDepth) > 20) {
    lastDepth = waterDepth;
    doc["de"] = lastDepth;
    sendJsonUpdate("de", doc);
  }
  // RSSI update
  int currentWiFiRssi = WiFi.RSSI();
  if (abs(currentWiFiRssi - lastWiFirssi) > 5) {
    lastWiFirssi = currentWiFiRssi;
    doc["wi"] = currentWiFiRssi;
    sendJsonUpdate("wi", doc);
  }
  // Inputs update
  bool inputsChanged = false;
  for (int i = 0; i < 4; i++) {
    if (lastInput[i] != input[i]) {
      lastInput[i] = input[i];
      inputsChanged = true;
    }
  }
  if (inputsChanged) {
    JsonArray inputs = doc.createNestedArray("ip");
    for (int i = 0; i < 4; i++) {
      inputs.add(lastInput[i]);
    }
    sendJsonUpdate("ip", doc);
  }
  // Outputs update
  bool outputsChanged = false;
  for (int i = 0; i < 4; i++) {
    if (lOutput[i] != output[i]) {
      lOutput[i] = output[i];
      outputsChanged = true;
    }
  }
  if (outputsChanged) {
    JsonArray outputs = doc.createNestedArray("ou");
    for (int i = 0; i < 4; i++) {
      outputs.add(!(output[i]));
    }
    sendJsonUpdate("ou", doc);
  }
  // Cleaning status update
  if (lastClean != cleaningOn) {
    lastClean = cleaningOn;
    doc["cl"] = lastClean;
    sendJsonUpdate("cl", doc);
  }
  // Flag update
  if (lastflagDrainOn != flagDrainOn) {
    lastflagDrainOn = flagDrainOn;
    doc["fl"] = lastflagDrainOn;
    sendJsonUpdate("fl", doc);
  }
}

void WiFiComm::sendJsonUpdate(const char* key, DynamicJsonDocument& value) {
  String jsonString; 
  // Create a new document to send
  DynamicJsonDocument doc(value.capacity()); // Using the same capacity as `value`
  doc[key] = value[key]; // Add key and value
  // Serialize a new document
  serializeJson(doc, jsonString);
  // Send JSON via WebSocket
  ws.textAll(jsonString);
  // Debugging
  Serial.println("JSON sent: " + jsonString);
}

void WiFiComm::sendLogFile() {
  File logFile = SD.open("/data.txt", FILE_READ);
  if (!logFile) {
    String message;
    message = String("couldNotOpenLogFile");
    WiCo.sendErrorMessage(message.c_str());
    Serial.println("Could not open log file");
    return;
  }
  // Store last 50 lines
  const int maxLines = 50;
  String lastLines[maxLines];
  int lineCount = 0;
  // Read all lines from the file
  while (logFile.available()) {
    String line = logFile.readStringUntil('\n');
    lastLines[lineCount % maxLines] = line;
    lineCount++;
  }
  logFile.close();
  // Determine the starting line
  int start = lineCount > maxLines ? lineCount - maxLines : 0;
  // Prepare JSON response
  DynamicJsonDocument jsonResponse(2048);
  JsonArray logsArray = jsonResponse.createNestedArray("lf");
  for (int i = 0; i < maxLines && start + i < lineCount; i++) {
    logsArray.add(lastLines[(start + i) % maxLines]);
  }
  String jsonResponseString;
  serializeJson(jsonResponse, jsonResponseString);
  ws.textAll(jsonResponseString);
  Serial.println("JSON sent: " + jsonResponseString);
}

void WiFiComm::sendErrorMessage(const char *errormessage) {
    String jsonMessage;
    jsonMessage = String("{\"error\":\"") + String(errormessage) + String("\"}");
    ws.textAll(jsonMessage);
    Serial.println(jsonMessage);
}

bool WiFiComm::sendWebPage() {
  if (!SPIFFS.begin(false)) {
    Serial.println("An error has occurred while mounting SPIFFS");
    return false;
  }
  Serial.println("SPIFFS mounted successfully");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasHeader("Authorization")) {
      // Missing Authorization header, return login prompt
      AsyncWebServerResponse *response = request->beginResponse(401, "text/plain", "Authentication Required");
      response->addHeader("WWW-Authenticate", "Basic realm=\"ESP32\"");
      request->send(response);
    } else {
      // Check the contents of the Authorization header
      String authHeader = request->header("Authorization");
      Serial.println("Authorization Header: " + authHeader);
        // Compare Base64 encoded credentials
        if (authHeader.equals("Basic " + String(expectedAuth))) {
          File file = SPIFFS.open("/Index.html", "r");
          if (!file) {
            Serial.println("Failed to open /Index.html");
            request->send(500, "text/plain", "Failed to open file");
            return;
          }
          String htmlContent = file.readString();
          file.close();
          request->send(200, "text/html", htmlContent);
        } else {
            request->send(403, "text/plain", "Forbidden: Invalid Credentials");
        }
      }
    });
  return true;
}
