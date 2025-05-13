# ESP32WebsocketServer

# Description

This project involves a control unit with an ESP32 microcontroller that manages a small wastewater treatment plant. It is powered by 24 V AC. A LM2596 DC/DC converter is used to provide 3.3V/DC for the processor and other components. USB communication is handled by an FT232 chip, and the MCP79410 with a CR2032 battery is selected for RTC. The control unit has 4 inputs and 4 outputs. Temperature measurement is implemented using a DS18B20 sensor connected to the Thermo input, and level measurement is done with an EARU pressure sensor. Each activity is logged to the SD Card and these logs can be retrieved via the "Čisticka" application. The device connects via WiFi to a home network, allowing control through a WebSocket server. The website is stored on SD card and is also accessible within the home network. The circuit schematic and PCB design can be downloaded from the website: [https://oshwlab.com](https://oshwlab.com/bobobo007/cisticka-_v06-001) (copy the link int new window)

![PCB](https://github.com/user-attachments/assets/32259623-8603-4f0c-9ad5-117dca7c9e11)

---

## Libraries Used

- [OneWire](https://github.com/PaulStoffregen/OneWire) (v2.3.8)
- [NTPClient](https://github.com/arduino-libraries/NTPClient) (v2.3.1)
- [FatFs](https://github.com/stm32duino/FatFs) (v4.0.0)
- [SD](https://docs.arduino.cc/libraries/sd/) (v1.3.0)
- [ESPAsyncWebServer](https://github.com/mathieucarbou/ESPAsyncWebServer) (v3.6.0)
- [AsyncTCP](https://github.com/mathieucarbou/AsyncTCP) (v3.3.2)
- [ArduinoJson](https://arduinojson.org/?utm_source=meta&utm_medium=library.properties) (v7.3.0)
- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library) (v3.9.1)

---

## Boards Used

- [esp32](https://github.com/espressif/arduino-esp32) (v3.1.1)
- [Arduino ESP32 Boards](https://github.com/espressif/arduino-esp32) (v2.0.18)

I hope I didn’t forget any.

Compiled with Arduino IDE v2.3.4.

---

## Setup Instructions

1. In the `data` folder, locate the `Index.html` file and set the WebSocket server address:
   ```javascript
   const ws = new WebSocket('ws://192.168.1.100/ws'); // Replace with your server address
   ```
   This address must match the one set in `WiFiComm.cpp`.

2. Upload the modified `Index.html` file to the ESP32's SPIFFS. I used Arduino IDE 1.8.19 with the "ESP32 Sketch Data Upload" tool installed.

3. In `WiFiComm.cpp`, set the username and password for authentication. Encode these using [Base64 Encode](https://www.base64encode.org/) in the format `username:password`. For example:
   ```cpp
   const char* expectedAuth = "YWRtaW46YWRtaW4="; // Base64 for "admin:admin"
   ```

4. Configure the WiFi network name and password:
   ```cpp
   const char* ssid     = "XYNetwork";
   const char* password = "123456789";
   ```

5. Set the IP Address, Subnet Mask, Gateway, and DNS server. Ensure the IP address matches the one used in `Index.html`.

6. In Arduino IDE, go to `Tools -> Board` and select `ESP32 Dev Module`.

7. Compile and upload the project to the ESP32. If everything is set up correctly, you can access the website stored in SPIFFS by entering the IP address in a browser.
   
![WebPage](https://github.com/user-attachments/assets/04ccbc9e-d84a-4eab-9c57-5cead834c2de)

---

## Next Step

Install the application using Android Studio. Coming later


