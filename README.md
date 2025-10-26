<p align="center">
  <img src="docs/logo.png" width="200">
</p>

<h1 align="center">ESP32 Wastewater Treatment Controller</h1>
<p align="center">
Smart ESP32-based controller for compact wastewater treatment systems.<br>
WebSocket • Android App • WiFi • SD Logging • PCB & Web UI
</p>

---

## ✔️ Overview

This project provides a complete hardware & software solution for controlling a compact **wastewater treatment system** using an **ESP32** microcontroller.  
The controller monitors and automates draining, recirculation, water level, heating, and system safety.

It can be fully controlled using:
- ✅ Android application (WebSocket client)
- ✅ Integrated ESP32 Webpage (SPIFFS)

PCB project at OSHWLab:  
🔗 https://oshwlab.com/bobobo007/cisticka-_v06-001

Main firmware repository:  
🔗 https://github.com/bobobo007/ESP32WebsocketServer

---

## ✨ Features

| Category | Description |
|---------|-------------|
| Connectivity | WiFi, WebSocket Server |
| Control Modes | Recirculation, Cleaning, Manual draining |
| Data Logging | SD card automatic logging |
| Temperature Sensors | Valve temperature + PCB temperature |
| Level Measurement | Pressure-based EARU sensor |
| Inputs | 4 digital inputs (tank states, valve feedback…) |
| Outputs | 4 controlled outputs (valves, heating) |
| Local UI | Responsive website stored in SPIFFS |
| Mobile App | Android application included |
| RTC Clock | MCP79410 + CR2032 backup |

---

## 🖥 Screenshots (Android App)

<p align="center">
  <img src="docs/screen_main.jpg" width="270">  
  <img src="docs/screen_info.jpg" width="270">  
  <img src="docs/screen_log.jpg" width="270">  
</p>

<p align="center">
  <img src="docs/screen_settings.jpg" width="270">
</p>

---

## 🌐 Web Interface (SPIFFS)

<p align="center">
  <img src="docs/web_main.jpg" width="550">
</p>

---

## 🧠 System Architecture

| Component | Type | Notes |
|----------|------|------|
| MCU | ESP32 | WebSocket communication & control |
| Power | 24V AC → LM2596 → 3.3V DC | Stabilized processor supply |
| RTC | MCP79410 | Backup battery CR2032 |
| USB Interface | FT232 | Firmware & debugging |
| SD Card | microSD | Event logging (FatFS) |
| Sensors | DS18B20 (2×), EARU pressure | Temperature & level |
| I/O | 4 inputs / 4 outputs | Valves + heaters |

PCB Rendering:

<p align="center">
  <img src="docs/pcb_render.jpg" width="600">
</p>

---

## 🔄 WebSocket Protocol

### Client → Server commands

| Command | Function | Example |
|--------|----------|---------|
| `hb` | Heartbeat every 3s | `{"com":"hb"}` |
| `gv` | Get current values | `{"com":"gv","sta":true}` |
| `cl` | Cleaning/Recirc mode | `{"com":"cl","sta":true}` |
| `dr` | Open drain valve | `{"com":"dr","sta":true}` |
| `st` | Set time from phone | `{"com":"st","time":"yyyy-MM-dd'T'HH:mm:ssXXX"}` |
| `nt` | Sync time via NTP | `{"com":"nt","sta":true}` |
| `sl` | Send log data | `{"com":"sl","sta":true}` |
| `dl` | Delete log | `{"com":"dl","sta":true}` |
| `ou` | Set output state | `{"com":"ou1","sta":true}` |

---

### Server → Client values example

```json
{
 "so":"V07.001","ha":"V07.001",
 "te":23.25,"tp":30.8125,"de":0,"wi":-88,
 "cl":false,"fl":false,
 "ip":[false,false,false,false],
 "ou":[false,false,false,false]
}
