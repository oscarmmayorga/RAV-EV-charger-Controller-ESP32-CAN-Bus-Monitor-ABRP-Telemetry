A dual-core ESP32 firmware that interfaces with two MCP2515 CAN bus controllers to decode, filter, and process real-time data from an electric vehicle's BMS and onboard charger. Telemetry is streamed live to A Better Route Planner (ABRP) and displayed on a responsive web dashboard accessible over WiFi.

Features
Dual CAN bus decoding (BMS + charger) running on Core 0, isolated from all network activity
Automatic contactor sequencing — drives charge enable sequences based on BMS and charger state machines
Live web dashboard (Core 1) — auto-refreshes every second showing SOC, HV voltage/current/power, cell min/max/diff voltages, cell temperatures, BMS power available (MaxRegen / MaxDis), gear, speed, charger state, and IGBT temperatures
ABRP telemetry — sends SOC, speed, power, voltage, current, battery temp, and charging state to the ABRP API every second
Smart WiFi management — WiFi turns on only when the BMS contactor is closed; tries STA mode for 30 seconds then falls back to AP (EV_controller / EVesp32)
HTTP Basic Auth — web interface protected with configurable admin password
Two-tab web UI
Status — live vehicle data
Config — WiFi credentials (with network scan), admin password, and ABRP API key / token; all persisted to NVS (non-volatile storage)
Hardware
ESP32
2× MCP2515 CAN bus modules (SPI)
Connected to vehicle CAN networks (BMS and charger)
Stack
Arduino framework for ESP32
FreeRTOS (dual-core task pinning)
mcp_can library
WebServer, HTTPClient, Preferences (ESP32 Arduino core)
