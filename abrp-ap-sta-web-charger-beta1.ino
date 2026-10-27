// ─────────────────────────────────────────────────────────────────────────────
//  INCLUDES
// ─────────────────────────────────────────────────────────────────────────────
#include <SPI.h>
#include <mcp_can.h>  //https://github.com/coryjfowler/MCP_CAN_lib
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <time.h>
#include <Preferences.h>   // NVS non-volatile storage

// ─────────────────────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────
struct CanMsg;
void sendCanMsg(MCP_CAN &bus, const CanMsg &m);
void runAutoMode();

// ─────────────────────────────────────────────────────────────────────────────
//  MCP2515 #1 — ECU
// ─────────────────────────────────────────────────────────────────────────────
#define CAN1_CS  5
#define CAN1_INT 4
MCP_CAN CAN1(CAN1_CS);

// ─────────────────────────────────────────────────────────────────────────────
//  MCP2515 #2 — Charger
// ─────────────────────────────────────────────────────────────────────────────
#define CAN2_CS  15
#define CAN2_INT 2
MCP_CAN CAN2(CAN2_CS);

// ─────────────────────────────────────────────────────────────────────────────
//  D13 — Enable pin (INPUT_PULLUP)
// ─────────────────────────────────────────────────────────────────────────────
#define ENABLE_PIN 13

// ─────────────────────────────────────────────────────────────────────────────
//  ABRP API credentials (API_KEY and TOKEN are NVS-backed and editable via web)
// ─────────────────────────────────────────────────────────────────────────────
const char* API_URL = "https://api.iternio.com/1/tlm/send";
char nvsApiKey[64]  = "a5e91557-d169-47eb-a8af-";  // default, overwritten by NVS
char nvsToken[64]   = "ecd465e7-ac91-4031-9e07-";  // default, overwritten by NVS

// ─────────────────────────────────────────────────────────────────────────────
//  AP fallback credentials (fixed)
// ─────────────────────────────────────────────────────────────────────────────
#define AP_SSID "ESP32-CAN"
#define AP_PASS "RAV4EV-CAN"

// ─────────────────────────────────────────────────────────────────────────────
//  NVS — persistent storage
// ─────────────────────────────────────────────────────────────────────────────
Preferences prefs;

// Runtime copies of NVS values (loaded at boot, updated from Config page)
char nvsWifiSSID[64]   = "iPhone";
char nvsWifiPass[64]   = "meaner1551";
char nvsAdminPass[32]  = "admin";   // default web password

// ─────────────────────────────────────────────────────────────────────────────
//  Web server (runs on core 1)
// ─────────────────────────────────────────────────────────────────────────────
WebServer server(80);

// ─────────────────────────────────────────────────────────────────────────────
//  SPI mutex — protects shared SPI bus between cores
// ─────────────────────────────────────────────────────────────────────────────
portMUX_TYPE spiMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────────────────────────────────────
//  FreeRTOS task handles
// ─────────────────────────────────────────────────────────────────────────────
TaskHandle_t taskCanHandle  = NULL;
TaskHandle_t taskWifiHandle = NULL;

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi state machine flags (written by core 0 CAN task, read by core 1 WiFi task)
// ─────────────────────────────────────────────────────────────────────────────
volatile bool wifiShouldBeOn  = true;   // true when BMS contactor == CLOSED
volatile bool wifiIsRunning   = false;   // set by WiFi task once server is up
volatile bool wifiIsAP        = false;   // true when running in AP fallback mode

// ─────────────────────────────────────────────────────────────────────────────
//  DBC NAME TABLES
// ─────────────────────────────────────────────────────────────────────────────
const char* bmsContactorStateName(uint8_t v) {
  switch(v){
    case 0: return "BMS_CTRSET_OPEN";
    case 1: return "BMS_CTRSET_PRECHARGE";
    case 2: return "BMS_CTRSET_CLOSED";
    case 3: return "BMS_CTRSET_OPENING";
    case 4: return "BMS_CTRSET_SNA";
    default: return "UNKNOWN";
  }
}
const char* chgMainStateName(uint8_t v) {
  switch(v){
    case 0: return "CHG_MAIN_STATE_STANDBY";
    case 1: return "CHG_MAIN_STATE_CLEAR_FAULT";
    case 2: return "CHG_MAIN_STATE_FAULT";
    case 3: return "CHG_MAIN_STATE_PRECHARGE";
    case 4: return "CHG_MAIN_STATE_ENABLE";
    case 5: return "CHG_MAIN_STATE_FC_STARTUP";
    case 6: return "CHG_MAIN_STATE_FC_ENABLE";
    default: return "UNKNOWN";
  }
}
const char* chgSubStateName(uint8_t v) {
  switch(v){
    case 0:  return "CHG_STATE_STARTUP";
    case 1:  return "CHG_STATE_IDLE";
    case 2:  return "CHG_STATE_TRANSITION_TO_PRECHARGE";
    case 3:  return "CHG_STATE_WAIT_ACCEPT";
    case 4:  return "CHG_STATE_ACCEPT";
    case 5:  return "CHG_STATE_HAVE_VLINE";
    case 6:  return "CHG_STATE_FAILED_TO_START";
    case 7:  return "CHG_STATE_PRECHARGE";
    case 8:  return "CHG_STATE_PRECHARGE_INITIAL_TEST";
    case 9:  return "CHG_STATE_PRECHARGE_WAIT_FINAL";
    case 10: return "CHG_STATE_PRECHARGE_FINAL_TEST";
    case 11: return "CHG_STATE_WAIT_ARM_PWM";
    case 12: return "CHG_STATE_ARM_PWM";
    case 13: return "CHG_STATE_CHECK_PWM_ENABLED";
    case 14: return "CHG_STATE_TRANSITION_TO_ENABLE";
    case 15: return "CHG_STATE_PWM_ON";
    case 16: return "CHG_STATE_ANTISPARK_ON";
    case 17: return "CHG_STATE_ANTISPARK_CLOSED";
    case 18: return "CHG_STATE_ENABLE_PID";
    case 19: return "CHG_STATE_PRECHARGE_OFF";
    case 20: return "CHG_STATE_RAMP_CURRENT";
    case 21: return "CHG_STATE_CHARGING";
    case 22: return "CHG_STATE_SOFTWARE_FAULT";
    case 23: return "CHG_STATE_FAULTED";
    case 24: return "CHG_STATE_CLRFLT";
    case 30: return "CHG_STATE_SOFT_SHUTDOWN";
    case 31: return "CHG_STATE_CURRENT_OFF";
    case 32: return "CHG_STATE_OPEN_RELAYS";
    default: return "CHG_STATE_UNKNOWN";
  }
}
const char* chgProximityName(uint8_t v) {
  switch(v){
    case 0: return "CHG_PROXIMITY_SNA";
    case 1: return "CHG_PROXIMITY_DISCONNECTED";
    case 2: return "CHG_PROXIMITY_UNLATCHED";
    case 3: return "CHG_PROXIMITY_LATCHED";
    default: return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN TX SEQUENCES
// ─────────────────────────────────────────────────────────────────────────────
struct CanMsg {
  uint32_t id;
  uint8_t  ext;
  uint8_t  dlc;
  uint8_t  data[8];
};
static const CanMsg MINIMUMRUN1[] = {
  { 0x05EC, 0, 8, { 0x43, 0x48, 0x47, 0x2D, 0x52, 0x39, 0x00, 0x00 } },
  { 0x021C, 0, 8, { 0x00, 0xEC, 0x00, 0x00, 0x7B, 0x01, 0x00, 0x04 } },
};
#define MINIMUMRUN1_LEN (sizeof(MINIMUMRUN1)/sizeof(MINIMUMRUN1[0]))

static const CanMsg CHARGING1[] = {
  { 0x212, 0, 5, { 0x40, 0x00, 0x00, 0xB5, 0x00, 0, 0, 0 } },
  { 0x222, 0, 6, { 0x00, 0x00, 0xD6, 0x90, 0x3C, 0x00, 0, 0 } },
  { 0x212, 0, 5, { 0x40, 0x00, 0x01, 0xB5, 0x00, 0, 0, 0 } },
  { 0x222, 0, 6, { 0x00, 0x00, 0xD6, 0x90, 0x3C, 0x00, 0, 0 } },
  { 0x212, 0, 5, { 0xD0, 0x00, 0x32, 0xB5, 0x00, 0, 0, 0 } },
  { 0x222, 0, 6, { 0x00, 0x00, 0xD6, 0x90, 0x3C, 0x04, 0, 0 } },
};
#define CHARGING1_LEN (sizeof(CHARGING1)/sizeof(CHARGING1[0]))

static const CanMsg CHARGING2[] = {
  { 0x00000358, 0, 5, { 0x00, 0x02, 0x28, 0x00, 0x00, 0,    0,    0x00 } },
  { 0x00000212, 0, 5, { 0xD0, 0x00, 0x32, 0xB5, 0x00, 0,    0,    0x00 } },
  { 0x00000348, 0, 8, { 0x49, 0x29, 0x00, 0x00, 0x00, 0,    0,    0x00 } },
  { 0x00000368, 0, 8, { 0x01, 0x00, 0x00, 0x00, 0x00, 0,    0,    0xFF } },
  { 0x0000036A, 0, 8, { 0xF8, 0x01, 0xF9, 0x01, 0x64, 0x64, 0,    0x00 } },
  { 0x000003A2, 0, 5, { 0x2E, 0x00, 0x2E, 0xA0, 0x06, 0,    0,    0x00 } },
  { 0x00000222, 0, 6, { 0xFC, 0x24, 0x14, 0x98, 0xA0, 0x12, 0,    0x00 } },
};
#define CHARGING2_LEN (sizeof(CHARGING2)/sizeof(CHARGING2[0]))

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBAL VARIABLES  (shared between cores; core 1 reads, core 0 writes)
// ─────────────────────────────────────────────────────────────────────────────
volatile float v_bat_voltage     = 0, v_bat_current = 0, v_bat_power = 0;
volatile float v_bat_soc         = 0, v_bat_energy_avail = 0;
volatile char  v_gear_letter     = '?';
volatile float v_speed           = 0;   // km/h (already converted x1.6)
volatile float v_chg_voltage     = 0, v_chg_power = 0, v_chg_current = 0;
volatile bool  v_charging        = false;
volatile float igbt              = 0, mag = 0;

volatile int8_t lastBmsContactorState = -1;
volatile int8_t lastChgMainState      = -1;
volatile int8_t lastChgSubState       = -1;
volatile int8_t lastChgProximity      = -1;

volatile float cellMinV  = 100.0;
volatile float cellMaxV  = 0.0;
volatile float cellDiffV = 100.0;
volatile float tempMin   = 100.0;
volatile float tempMax   = -100.0;
volatile float tempAvg   = 0;

// BMS_powerAvailable (0x232)
volatile float bmsMaxRegen = 0;
volatile float bmsMaxDis   = 0;

volatile uint8_t autosecuence = 0;
volatile bool    autoModeOn   = false;

// Last ABRP HTTP response code (shown on web dashboard)
volatile int lastAbrpCode = 0;

// CAN task internal timer
unsigned long lastPrint = 0;

// TX sequence indices
uint8_t idxMinRun    = 0;
uint8_t idxCharging1 = 0;
uint8_t idxCharging2 = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  CAN HELPERS
// ─────────────────────────────────────────────────────────────────────────────
void updateMinMax(float &minV, float &maxV, float val) {
  if (val < minV) minV = val;
  if (val > maxV) maxV = val;
}

void sendCanMsg(MCP_CAN &bus, const CanMsg &m) {
  bus.sendMsgBuf(m.id, m.ext, m.dlc, (uint8_t*)m.data);
}

void decodeCAN(MCP_CAN &mcp) {
  long unsigned int rxId;
  unsigned char len;
  unsigned char d[8];
  mcp.readMsgBuf(&rxId, &len, d);

  switch (rxId) {

    case 0x102: {
      if (len < 4) break;
      float fPV = (float)(((int)d[1]<<8)+d[0]) / 100.0f;
      int   iPA = (((int)(d[3]&0x3F))<<8) + d[2];
      float fPA = (d[3]&0x40) ? (-0.1f*(float)(iPA-16384)) : (-0.1f*(float)iPA);
      v_bat_voltage = fPV;
      v_bat_current = fPA;
      v_bat_power   = fPV * fPA / 1000.0f;
      if (lastChgMainState==4&&autoModeOn){
        v_bat_power   =v_bat_power   - v_chg_power;
      }
      break;
    }

    case 0x302: {
      if (len < 5) break;
      v_bat_soc          = (float)((((int)d[1]&0x03)<<8)+d[0]) / 10.0f;
      v_bat_energy_avail = (float)((((int)d[4]&0x0F)<<6)+((d[3]&0xFC)>>2)) / 10.0f;
      break;
    }

    case 0x116: {
      if (len < 2) break;
      switch ((d[1]&0x70)>>4) {
        case 1: v_gear_letter='P'; break;
        case 2: v_gear_letter='R'; break;
        case 3: v_gear_letter='N'; break;
        case 4: v_gear_letter='D'; break;
        case 5: v_gear_letter='B'; break;
      }
      break;
    }

    case 0x256: {
      if (len < 4) break;
      // Raw value is in mph — multiply by 1.6 to convert to km/h
      v_speed = (float)((((int)d[3] & 0x0F) << 8) + (int)d[2]) / 10.0f * 1.6f;
      break;
    }

    case 0x212: {
      if (len < 3) break;
      lastBmsContactorState = d[2] & 0x07;
      break;
    }

    case 0x20C: {
      if (len < 4) break;
      v_chg_voltage = (float)(((d[2]&0x01)<<8)+d[1]);
      v_chg_power   = (float)((d[3]<<3)+((d[2]&0xE0)>>5)) * 0.01f;
      v_chg_current = (v_chg_voltage > 0) ? (v_chg_power*1000.0f/v_chg_voltage) : 0;
      break;
    }

    case 0x21C: {
      if (len < 3) break;
      lastChgMainState = d[0] >> 4;
      if (len >= 6) lastChgSubState = d[5] & 0x3F;
      lastChgProximity = (d[1]&0x18)>>3;
      break;
    }

    case 0x232: {
      // BMS_powerAvailable — store maxRegen and maxDis for dashboard display
      if (len < 4) break;
      bmsMaxRegen = ((d[1]<<8)+d[0]) * 0.01f;
      bmsMaxDis   = ((d[3]<<8)+d[2]) * 0.01f;
      break;
    }

    case 0x23C: {
      if (len < 2) break;
      igbt = d[0] - 40;
      mag  = d[1] - 40;
      break;
    }

    case 0x6F2: {
      int v1 = ((d[2]&0x3f)<<8) + d[1];
      int v2 = (((int)d[4]&0x0f)<<10) + (d[3]<<2) + (d[2]>>6);
      int v3 = (((int)d[6]&0x03)<<12) + (d[5]<<4) + (d[4]>>4);
      int v4 = ((d[7])<<6) + (d[6]>>2);

      if (d[0] == 0)  { cellMinV = 100; cellMaxV = 0; }
      else if (d[0] == 24) { tempMin = 100; tempMax = -100; }

      if (d[0] < 24) {
        float c1 = 0.000305f*v1, c2 = 0.000305f*v2;
        float c3 = 0.000305f*v3, c4 = 0.000305f*v4;
        float cMin = cellMinV, cMax = cellMaxV;
        updateMinMax(cMin, cMax, c1); updateMinMax(cMin, cMax, c2);
        updateMinMax(cMin, cMax, c3); updateMinMax(cMin, cMax, c4);
        cellMinV = cMin; cellMaxV = cMax;
      } else {
        float t1 = 0.0122f*((v1&0x1FFF)-(v1&0x2000));
        float t2 = 0.0122f*((v2&0x1FFF)-(v2&0x2000));
        float t3 = 0.0122f*((v3&0x1FFF)-(v3&0x2000));
        float t4 = 0.0122f*((v4&0x1FFF)-(v4&0x2000));
        float tMin = tempMin, tMax = tempMax;
        updateMinMax(tMin, tMax, t1); updateMinMax(tMin, tMax, t2);
        updateMinMax(tMin, tMax, t3); updateMinMax(tMin, tMax, t4);
        tempMin = tMin; tempMax = tMax;
      }
      cellDiffV = cellMaxV - cellMinV;
      tempAvg   = (tempMin + tempMax) / 2.0f;
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  AUTO MODE TX
// ─────────────────────────────────────────────────────────────────────────────
void runAutoMode() {
  bool id212_open   = (lastBmsContactorState == 0);
  bool id212_closed = (lastBmsContactorState == 2);

  if (id212_open) {
    sendCanMsg(CAN1, MINIMUMRUN1[idxMinRun]);
    idxMinRun = (idxMinRun + 1) % MINIMUMRUN1_LEN;
    idxCharging1 = 0; idxCharging2 = 0; autosecuence = 0;
    return;
  }
  if (id212_closed) {
    
    idxMinRun = 0;
    bool id21C_fault   = (lastChgMainState == 2);
    bool id21C_noState = (lastChgMainState == -1);
    autosecuence = 1;
    if (id21C_fault || id21C_noState) {
      sendCanMsg(CAN2, CHARGING1[idxCharging1]);
      if (idxCharging1==0)
        sendCanMsg(CAN1, MINIMUMRUN1[1]);
      idxCharging1 = (idxCharging1 + 1) % CHARGING1_LEN;
      idxCharging2 = 0; autosecuence = 2;
      return;
    } else if (lastChgProximity == 3) {
      sendCanMsg(CAN2, CHARGING2[idxCharging2]);
      if (idxCharging2==0)
        sendCanMsg(CAN1, MINIMUMRUN1[1]);
      idxCharging2 = (idxCharging2 + 1) % CHARGING2_LEN;
      idxCharging1 = 0; autosecuence = 3;
      return;
    } else{
      sendCanMsg(CAN1, MINIMUMRUN1[1]);
      return;
    }
    
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CORE 0 TASK — CAN bus (identical logic to original loop())
// ─────────────────────────────────────────────────────────────────────────────
void taskCAN(void* pvParameters) {
  while (true) {

    // ---------- RX CAN1 ----------
    if (!digitalRead(CAN1_INT)) {
      portENTER_CRITICAL(&spiMux);
      decodeCAN(CAN1);
      portEXIT_CRITICAL(&spiMux);
    }

    // ---------- RX CAN2 ----------
    if (!digitalRead(CAN2_INT)) {
      portENTER_CRITICAL(&spiMux);
      decodeCAN(CAN2);
      portEXIT_CRITICAL(&spiMux);
    }

    // ---------- AUTO MODE (D13 HIGH = active) ----------
    if (digitalRead(ENABLE_PIN) == HIGH) {
      autoModeOn = true;
      portENTER_CRITICAL(&spiMux);
      runAutoMode();
      portEXIT_CRITICAL(&spiMux);
    } else {
      autoModeOn = false;
    }

    // ---------- WiFi enable/disable based on BMS contactor state ----------
    // WiFi should be ON only when BMS contactor is CLOSED (state 2)
    wifiShouldBeOn = (lastBmsContactorState == 2);

    // ---------- Serial print every 1 second ----------
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      if (autoModeOn) {
        Serial.println("automode on");
        Serial.printf("auto sequence number: %d\n", (int)autosecuence);
      } else {
        Serial.println("automode off");
      }
      Serial.println("------ DATA ------");
      Serial.printf("SOC: %.1f %%  Energy: %.1f kWh\n",             (float)v_bat_soc, (float)v_bat_energy_avail);
      Serial.printf("HV: %.1f V  %.1f A  %.1f kW\n",                (float)v_bat_voltage, (float)v_bat_current, (float)v_bat_power);
      Serial.printf("Cells: Min %.3f V  Max %.3f V  Diff %.3f V\n", (float)cellMinV, (float)cellMaxV, (float)cellDiffV);
      Serial.printf("Cells Temp: Min %.1f C  Max %.1f C  Avg %.1f C\n", (float)tempMin, (float)tempMax, (float)tempAvg);
      Serial.printf("BMS Limits: MaxRegen %.2f kW  MaxDis %.2f kW\n",(float)bmsMaxRegen, (float)bmsMaxDis);
      Serial.printf("Contactor: %s  CHG Main: %s  Sub: %s\n",
        bmsContactorStateName(lastBmsContactorState),
        chgMainStateName(lastChgMainState),
        chgSubStateName(lastChgSubState));
      Serial.printf("Gear: %c\n",                                    (char)v_gear_letter);
      Serial.printf("Speed: %.1f km/h\n",                            (float)v_speed);
      Serial.printf("Charger: %.1f V  %.1f kW  %.1f A\n",           (float)v_chg_voltage, (float)v_chg_power, (float)v_chg_current);
      Serial.printf("Charger IGBT Temp: %.1f C  Magnetics Temp: %.1f C\n", (float)igbt, (float)mag);
      Serial.println();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  NTP SYNC
// ─────────────────────────────────────────────────────────────────────────────
void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP sync");
  time_t now = time(nullptr);
  while (now < 1000000000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println(" OK");
}

// ─────────────────────────────────────────────────────────────────────────────
//  ABRP SEND
// ─────────────────────────────────────────────────────────────────────────────
void sendToABRP() {
  time_t now = time(nullptr);

  // is_charging = 1 when gear is P AND charger main state == ENABLE (4)
  int isCharging = ((v_gear_letter == 'P') && (v_bat_power < 0)) ? 1 : 0;

  String json = "{";
  json += "\"utc\":"        + String((long)now)                    + ",";
  json += "\"soc\":"        + String((float)v_bat_soc, 1)          + ",";
  json += "\"soe\":"        + String((float)v_bat_energy_avail, 1) + ",";
  json += "\"speed\":"      + String((float)v_speed, 1)            + ",";
  json += "\"is_charging\":" + String(isCharging)                  + ",";
  json += "\"power\":"      + String((float)v_bat_power, 1)        + ",";
  json += "\"batt_temp\":"  + String((float)tempAvg, 1)            + ",";
  json += "\"current\":"    + String((float)v_bat_current, 1)      + ",";
  json += "\"voltage\":"    + String((float)v_bat_voltage, 1);
  json += "}";

  String url = String(API_URL) +
               "?api_key=" + String(nvsApiKey) +
               "&token="   + String(nvsToken)  +
               "&tlm="     + json;

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  lastAbrpCode = httpCode;
  http.end();
  Serial.printf("ABRP send: %d\n", httpCode);
}

// ─────────────────────────────────────────────────────────────────────────────
//  HTTP BASIC AUTH HELPER
// ─────────────────────────────────────────────────────────────────────────────
bool checkAuth() {
  if (!server.authenticate("admin", nvsAdminPass)) {
    server.requestAuthentication(BASIC_AUTH, "EV Monitor", "Unauthorized");
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  WEB — /data  (JSON endpoint polled every second by the dashboard)
// ─────────────────────────────────────────────────────────────────────────────
void handleData() {
  if (!checkAuth()) return;
  char buf[1200];
  snprintf(buf, sizeof(buf),
    "{"
    "\"automode\":%s,"
    "\"seq\":%d,"
    "\"soc\":%.1f,"
    "\"soe\":%.1f,"
    "\"volt\":%.1f,"
    "\"amp\":%.1f,"
    "\"kw\":%.2f,"
    "\"gear\":\"%c\","
    "\"speed\":%.1f,"
    "\"cmin\":%.4f,"
    "\"cmax\":%.4f,"
    "\"cdiff\":%.4f,"
    "\"tmin\":%.1f,"
    "\"tmax\":%.1f,"
    "\"tavg\":%.1f,"
    "\"maxRegen\":%.2f,"
    "\"maxDis\":%.2f,"
    "\"contactor\":\"%s\","
    "\"chgmain\":\"%s\","
    "\"chgsub\":\"%s\","
    "\"chgv\":%.1f,"
    "\"chgkw\":%.2f,"
    "\"chga\":%.1f,"
    "\"igbt\":%.1f,"
    "\"mag\":%.1f,"
    "\"abrp\":%d,"
    "\"wifiMode\":\"%s\""
    "}",
    autoModeOn ? "true" : "false",
    (int)autosecuence,
    (float)v_bat_soc,
    (float)v_bat_energy_avail,
    (float)v_bat_voltage,
    (float)v_bat_current,
    (float)v_bat_power,
    (char)v_gear_letter,
    (float)v_speed,
    (float)cellMinV,
    (float)cellMaxV,
    (float)cellDiffV,
    (float)tempMin,
    (float)tempMax,
    (float)tempAvg,
    (float)bmsMaxRegen,
    (float)bmsMaxDis,
    bmsContactorStateName(lastBmsContactorState),
    chgMainStateName(lastChgMainState),
    chgSubStateName(lastChgSubState),
    (float)v_chg_voltage,
    (float)v_chg_power,
    (float)v_chg_current,
    (float)igbt,
    (float)mag,
    (int)lastAbrpCode,
    wifiIsAP ? "AP" : "STA"
  );
  server.send(200, "application/json", buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  WEB — /scan  (returns JSON array of nearby SSIDs)
// ─────────────────────────────────────────────────────────────────────────────
void handleScan() {
  if (!checkAuth()) return;
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "\"" + WiFi.SSID(i) + "\"";
  }
  json += "]";
  server.send(200, "application/json", json);
}
// ─────────────────────────────────────────────────────────────────────────────
//  WIFI CONNECT HELPER — tries STA for 60 s then falls back to AP
// ─────────────────────────────────────────────────────────────────────────────
void startWifi() {
  wifiIsAP = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(nvsWifiSSID, nvsWifiPass);
  Serial.printf("Connecting to WiFi: %s\n", nvsWifiSSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 60000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected — IP: %s\n", WiFi.localIP().toString().c_str());
    setupTime();
    wifiIsAP = false;
  } else {
    // 30-second timeout — switch to AP mode
    Serial.println("WiFi timeout — starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP started — IP: %s\n", WiFi.softAPIP().toString().c_str());
    wifiIsAP = true;
  }
}
// ─────────────────────────────────────────────────────────────────────────────
//  WEB — /save_wifi  (POST: ssid + pass — saves to NVS and reconnects)
// ─────────────────────────────────────────────────────────────────────────────
void handleSaveWifi() {
  if (!checkAuth()) return;
  if (server.hasArg("ssid") && server.hasArg("pass")) {
    String s = server.arg("ssid");
    String p = server.arg("pass");
    s.toCharArray(nvsWifiSSID, sizeof(nvsWifiSSID));
    p.toCharArray(nvsWifiPass, sizeof(nvsWifiPass));
    prefs.begin("ev_cfg", false);
    prefs.putString("wifiSSID", s);
    prefs.putString("wifiPass", p);
    prefs.end();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"WiFi credentials saved. Reconnecting...\"}");
    // Trigger reconnect on next WiFi task cycle
    //wifiIsRunning = false;
    startWifi();
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing ssid or pass\"}");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  WEB — /save_adminpass  (POST: newpass — saves to NVS)
// ─────────────────────────────────────────────────────────────────────────────
void handleSaveAdminPass() {
  if (!checkAuth()) return;
  if (server.hasArg("newpass")) {
    String np = server.arg("newpass");
    np.toCharArray(nvsAdminPass, sizeof(nvsAdminPass));
    prefs.begin("ev_cfg", false);
    prefs.putString("adminPass", np);
    prefs.end();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Admin password updated.\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing newpass\"}");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  WEB — /save_abrp  (POST: apikey + token — saves to NVS)
// ─────────────────────────────────────────────────────────────────────────────
void handleSaveAbrp() {
  if (!checkAuth()) return;
  if (server.hasArg("apikey") && server.hasArg("token")) {
    String k = server.arg("apikey");
    String t = server.arg("token");
    k.toCharArray(nvsApiKey, sizeof(nvsApiKey));
    t.toCharArray(nvsToken,  sizeof(nvsToken));
    prefs.begin("ev_cfg", false);
    prefs.putString("abrpApiKey", k);
    prefs.putString("abrpToken",  t);
    prefs.end();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"ABRP credentials saved.\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing apikey or token\"}");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  WEB — / (main page: two tabs — Status and Config)
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
  if (!checkAuth()) return;

  // Build the full page as a raw string.
  // Tab 1: live dashboard (Status)
  // Tab 2: WiFi + admin password config (Config)
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RAV4 EV Controller</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh;padding:12px}
    h1{text-align:center;font-size:1.3rem;margin-bottom:10px;color:#58a6ff;letter-spacing:2px}

    /* TABS */
    .tabs{display:flex;gap:4px;max-width:640px;margin:0 auto 12px}
    .tab-btn{flex:1;padding:8px;border:1px solid #30363d;background:#161b22;color:#8b949e;
             border-radius:8px 8px 0 0;cursor:pointer;font-size:.9rem;transition:.2s}
    .tab-btn.active{background:#1f6feb;color:#fff;border-color:#1f6feb}
    .tab-content{display:none;max-width:640px;margin:auto}
    .tab-content.active{display:block}

    /* CARDS */
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
    .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:10px 14px}
    .card.full{grid-column:1/-1}
    .label{font-size:.65rem;color:#8b949e;text-transform:uppercase;letter-spacing:1px;margin-bottom:3px}
    .value{font-size:1.2rem;font-weight:700;color:#f0f6fc}
    .value.big{font-size:2rem;color:#3fb950}
    .value.ok{color:#3fb950}.value.warn{color:#d29922}.value.err{color:#f85149}
    .sub{font-size:.75rem;color:#8b949e;margin-top:2px}
    .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;background:#3fb950}
    .dot.off{background:#f85149}
    #status-bar{text-align:center;font-size:.65rem;color:#484f58;margin-top:10px}

    /* CONFIG FORM */
    .cfg-section{background:#161b22;border:1px solid #30363d;border-radius:10px;
                 padding:14px;margin-bottom:10px}
    .cfg-section h3{font-size:.85rem;color:#58a6ff;margin-bottom:10px;letter-spacing:1px}
    .cfg-row{display:flex;flex-direction:column;gap:4px;margin-bottom:8px}
    .cfg-row label{font-size:.7rem;color:#8b949e;text-transform:uppercase}
    .cfg-row input{background:#0d1117;border:1px solid #30363d;color:#e6edf3;
                   padding:7px 10px;border-radius:6px;font-size:.9rem;width:100%}
    .cfg-row input:focus{outline:none;border-color:#1f6feb}
    .btn{padding:8px 16px;border:none;border-radius:6px;cursor:pointer;
         font-size:.85rem;font-weight:600;margin-top:4px}
    .btn-primary{background:#1f6feb;color:#fff}
    .btn-secondary{background:#21262d;color:#c9d1d9;border:1px solid #30363d}
    .btn-sm{padding:6px 12px;font-size:.78rem}
    #ssid-list{display:none;background:#0d1117;border:1px solid #30363d;border-radius:6px;
               margin-top:6px;max-height:150px;overflow-y:auto}
    .ssid-item{padding:7px 10px;cursor:pointer;font-size:.85rem;border-bottom:1px solid #21262d}
    .ssid-item:hover{background:#21262d}
    .msg{font-size:.78rem;margin-top:6px;padding:6px 10px;border-radius:6px;display:none}
    .msg.ok{background:#0f2c14;color:#3fb950;border:1px solid #3fb950}
    .msg.err{background:#2c0f0f;color:#f85149;border:1px solid #f85149}
  </style>
</head>
<body>
  <h1>&#9889; Rav4 EV Controller</h1>

  <div class="tabs">
    <button class="tab-btn active" onclick="showTab('status')">Status</button>
    <button class="tab-btn"        onclick="showTab('config')">Config</button>
  </div>

  <!-- ══════════════════  TAB 1: STATUS  ══════════════════ -->
  <div class="tab-content active" id="tab-status">
    <div class="grid">
      <div class="card full">
        <div class="label">Mode</div>
        <div class="value" id="automode">—</div>
        <div class="sub" id="seq"></div>
      </div>
      <div class="card full">
        <div class="label">SOC</div>
        <div class="value big" id="soc">—</div>
        <div class="sub" id="energy"></div>
      </div>
      <div class="card">
        <div class="label">HV Voltage</div>
        <div class="value" id="voltage">—</div>
      </div>
      <div class="card">
        <div class="label">HV Current</div>
        <div class="value" id="current">—</div>
      </div>
      <div class="card full">
        <div class="label">HV Power</div>
        <div class="value" id="power">—</div>
      </div>
      <div class="card">
        <div class="label">Gear</div>
        <div class="value" id="gear">—</div>
      </div>
      <div class="card">
        <div class="label">Speed</div>
        <div class="value" id="speed">—</div>
      </div>
      <div class="card full">
        <div class="label">Cells (min / max / diff)</div>
        <div class="value" id="cells">—</div>
      </div>
      <div class="card full">
        <div class="label">Cell Temp (min / max / avg)</div>
        <div class="value" id="temps">—</div>
      </div>
      <div class="card full">
        <div class="label">BMS Power Available</div>
        <div class="value" id="bmspower">—</div>
        <div class="sub">MaxRegen / MaxDis</div>
      </div>
      <div class="card full">
        <div class="label">Contactor / CHG Main / Sub</div>
        <div class="value" id="contactor" style="font-size:.82rem">—</div>
      </div>
      <div class="card">
        <div class="label">Charger V / kW / A</div>
        <div class="value" id="chg">—</div>
      </div>
      <div class="card">
        <div class="label">IGBT / Magnetics Temp</div>
        <div class="value" id="igbt">—</div>
      </div>
      <div class="card full">
        <div class="label">ABRP HTTP Code</div>
        <div class="value" id="abrp">—</div>
      </div>
      <div class="card full">
        <div class="label">WiFi Mode</div>
        <div class="value" id="wifimode">—</div>
      </div>
    </div>
    <div id="status-bar">Loading...</div>
  </div>

  <!-- ══════════════════  TAB 2: CONFIG  ══════════════════ -->
  <div class="tab-content" id="tab-config">

    <!-- Admin password -->
    <div class="cfg-section">
      <h3>&#128274; Admin Password</h3>
      <div class="cfg-row">
        <label>New Password</label>
        <input type="password" id="newpass" placeholder="Enter new password">
      </div>
      <button class="btn btn-primary btn-sm" onclick="saveAdminPass()">Save Password</button>
      <div class="msg" id="msg-adminpass"></div>
    </div>

    <!-- WiFi credentials -->
    <div class="cfg-section">
      <h3>&#128246; WiFi Settings</h3>
      <div class="cfg-row">
        <label>SSID</label>
        <input type="text" id="cfg-ssid" placeholder="Network name">
        <div id="ssid-list"></div>
      </div>
      <button class="btn btn-secondary btn-sm" onclick="scanSSID()" style="margin-bottom:8px">
        &#128269; Scan Networks
      </button>
      <div class="cfg-row">
        <label>Password</label>
        <input type="password" id="cfg-wifipass" placeholder="Network password">
      </div>
      <button class="btn btn-primary btn-sm" onclick="saveWifi()">Connect</button>
      <div class="msg" id="msg-wifi"></div>
    </div>

    <!-- ABRP credentials -->
    <div class="cfg-section">
      <h3>&#128225; ABRP Credentials</h3>
      <div class="cfg-row">
        <label>API Key</label>
        <input type="text" id="cfg-apikey" placeholder="ABRP API Key">
      </div>
      <div class="cfg-row">
        <label>Token</label>
        <input type="text" id="cfg-token" placeholder="ABRP Vehicle Token">
      </div>
      <button class="btn btn-primary btn-sm" onclick="saveAbrp()">Save ABRP</button>
      <div class="msg" id="msg-abrp"></div>
    </div>

  </div><!-- /tab-config -->

  <script>
    // ── Tab switching ────────────────────────────────────────────────────────
    function showTab(id) {
      document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
      document.getElementById('tab-' + id).classList.add('active');
      event.target.classList.add('active');
    }

    // ── Status dashboard auto-refresh ────────────────────────────────────────
    async function update() {
      try {
        const r = await fetch('/data');
        if (r.status === 401) { document.getElementById('status-bar').textContent = 'Session expired — reload page'; return; }
        const d = await r.json();
        document.getElementById('automode').innerHTML =
          '<span class="dot' + (d.automode ? '' : ' off') + '"></span>' +
          (d.automode ? 'Automode ON' : 'Automode OFF');
        document.getElementById('seq').textContent = d.automode ? 'Sequence: ' + d.seq : '';
        document.getElementById('soc').textContent    = d.soc.toFixed(1) + ' %';
        document.getElementById('energy').textContent = 'Energy: ' + d.soe.toFixed(1) + ' kWh';
        document.getElementById('voltage').textContent= d.volt.toFixed(1) + ' V';
        document.getElementById('current').textContent= d.amp.toFixed(1) + ' A';
        document.getElementById('power').textContent  = d.kw.toFixed(2) + ' kW';
        document.getElementById('gear').textContent   = d.gear;
        document.getElementById('speed').textContent  = d.speed.toFixed(1) + ' km/h';
        document.getElementById('cells').textContent  =
          d.cmin.toFixed(3)+' V  /  '+d.cmax.toFixed(3)+' V  /  '+d.cdiff.toFixed(3)+' V';
        document.getElementById('temps').textContent  =
          d.tmin.toFixed(1)+' °C  /  '+d.tmax.toFixed(1)+' °C  /  '+d.tavg.toFixed(1)+' °C';
        document.getElementById('bmspower').textContent =
          d.maxRegen.toFixed(2) + ' kW  /  ' + d.maxDis.toFixed(2) + ' kW';
        document.getElementById('contactor').textContent =
          d.contactor + '  |  ' + d.chgmain + '  |  ' + d.chgsub;
        document.getElementById('chg').textContent =
          d.chgv.toFixed(0)+' V  '+d.chgkw.toFixed(2)+' kW  '+d.chga.toFixed(1)+' A';
        document.getElementById('igbt').textContent =
          d.igbt.toFixed(1)+' °C  /  '+d.mag.toFixed(1)+' °C';
        const code = d.abrp;
        const el = document.getElementById('abrp');
        el.textContent = code;
        el.className = 'value ' + (code === 200 ? 'ok' : code < 0 ? 'err' : 'warn');
        document.getElementById('wifimode').textContent = d.wifiMode === 'AP' ? 'Access Point (AP fallback)' : 'Station (connected)';
        document.getElementById('status-bar').textContent = 'Last update: ' + new Date().toLocaleTimeString();
      } catch(e) {
        document.getElementById('status-bar').textContent = 'Connection error';
      }
    }
    update();
    setInterval(update, 1000);

    // ── Config helpers ────────────────────────────────────────────────────────
    function showMsg(id, text, isOk) {
      const el = document.getElementById(id);
      el.textContent = text;
      el.className = 'msg ' + (isOk ? 'ok' : 'err');
      el.style.display = 'block';
      setTimeout(() => { el.style.display = 'none'; }, 4000);
    }

    async function saveAdminPass() {
      const np = document.getElementById('newpass').value.trim();
      if (!np) { showMsg('msg-adminpass','Password cannot be empty.',false); return; }
      const r = await fetch('/save_adminpass', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'newpass='+encodeURIComponent(np)
      });
      const j = await r.json();
      showMsg('msg-adminpass', j.msg, j.ok);
      if (j.ok) document.getElementById('newpass').value = '';
    }

    async function scanSSID() {
      const list = document.getElementById('ssid-list');
      list.style.display = 'block';
      list.innerHTML = '<div class="ssid-item" style="color:#8b949e">Scanning...</div>';
      try {
        const r = await fetch('/scan');
        const nets = await r.json();
        list.innerHTML = nets.map(s =>
          '<div class="ssid-item" onclick="selectSSID(\''+s.replace(/'/g,"\\'")+'\')">' + s + '</div>'
        ).join('');
      } catch(e) { list.innerHTML = '<div class="ssid-item" style="color:#f85149">Scan failed</div>'; }
    }

    function selectSSID(name) {
      document.getElementById('cfg-ssid').value = name;
      document.getElementById('ssid-list').style.display = 'none';
    }

    async function saveWifi() {
      const ssid = document.getElementById('cfg-ssid').value.trim();
      const pass = document.getElementById('cfg-wifipass').value;
      if (!ssid) { showMsg('msg-wifi','SSID cannot be empty.',false); return; }
      const r = await fetch('/save_wifi', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)
      });
      const j = await r.json();
      showMsg('msg-wifi', j.msg, j.ok);
    }

    async function saveAbrp() {
      const apikey = document.getElementById('cfg-apikey').value.trim();
      const token  = document.getElementById('cfg-token').value.trim();
      if (!apikey || !token) { showMsg('msg-abrp','API Key and Token cannot be empty.',false); return; }
      const r = await fetch('/save_abrp', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'apikey='+encodeURIComponent(apikey)+'&token='+encodeURIComponent(token)
      });
      const j = await r.json();
      showMsg('msg-abrp', j.msg, j.ok);
      if (j.ok) {
        document.getElementById('cfg-apikey').value = '';
        document.getElementById('cfg-token').value  = '';
      }
    }
  </script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}



// ─────────────────────────────────────────────────────────────────────────────
//  CORE 1 TASK — WiFi + Web server + ABRP (every 1 second)
// ─────────────────────────────────────────────────────────────────────────────
void taskWifi(void* pvParameters) {

  bool serverStarted   = false;
  bool prevShouldBeOn  = false;

  unsigned long lastAbrp = 0;

  while (true) {

    bool shouldBeOn = wifiShouldBeOn;   // read volatile flag set by CAN task

    // ── Contactor just CLOSED → start WiFi ──────────────────────────────────
    if (shouldBeOn && !prevShouldBeOn) {
      Serial.println("BMS contactor CLOSED — starting WiFi");
      startWifi();

      if (!serverStarted) {
        server.on("/",               handleRoot);
        server.on("/data",           handleData);
        server.on("/scan",           handleScan);
        server.on("/save_wifi",      handleSaveWifi);
        server.on("/save_adminpass", handleSaveAdminPass);
        server.on("/save_abrp",      handleSaveAbrp);
        server.begin();
        serverStarted = true;
        Serial.println("Web server started");
      }
      wifiIsRunning = true;
    }

    // ── Contactor left CLOSED → stop WiFi ───────────────────────────────────
    if (!shouldBeOn && prevShouldBeOn) {
      Serial.println("BMS contactor not CLOSED — stopping WiFi");
      server.stop();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiIsRunning = false;
      serverStarted = false;
      lastAbrpCode  = 0;
    }

    prevShouldBeOn = shouldBeOn;

    // ── Normal running: handle HTTP clients and send to ABRP ────────────────
    if (shouldBeOn && wifiIsRunning) {
      server.handleClient();

      if (millis() - lastAbrp >= 1000) {
        lastAbrp = millis();
        // Send ABRP only in STA mode (needs internet)
        if (!wifiIsAP && WiFi.status() == WL_CONNECTED) {
          sendToABRP();
        }
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  SPI.begin();

  pinMode(ENABLE_PIN, INPUT_PULLUP);

  // ── Load persistent settings from NVS ─────────────────────────────────────
  prefs.begin("ev_cfg", true);   // read-only open
  String s = prefs.getString("wifiSSID",   String(nvsWifiSSID));
  String p = prefs.getString("wifiPass",   String(nvsWifiPass));
  String a = prefs.getString("adminPass",  String(nvsAdminPass));
  String k = prefs.getString("abrpApiKey", String(nvsApiKey));
  String t = prefs.getString("abrpToken",  String(nvsToken));
  prefs.end();
  s.toCharArray(nvsWifiSSID,  sizeof(nvsWifiSSID));
  p.toCharArray(nvsWifiPass,  sizeof(nvsWifiPass));
  a.toCharArray(nvsAdminPass, sizeof(nvsAdminPass));
  k.toCharArray(nvsApiKey,    sizeof(nvsApiKey));
  t.toCharArray(nvsToken,     sizeof(nvsToken));
  Serial.printf("NVS loaded — SSID: %s  AdminPass: %s\n", nvsWifiSSID, nvsAdminPass);

  // ── CAN1 init ─────────────────────────────────────────────────────────────
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN1 OK");
  } else {
    Serial.println("CAN1 FAIL");
   // while(1);
  }
  CAN1.init_Mask(0, 0, 0x07FF0000);
  CAN1.init_Mask(1, 0, 0x07FF0000);
  CAN1.init_Filt(0, 0, 0x06F20000);
  CAN1.init_Filt(1, 0, 0x01020000);
  CAN1.init_Filt(2, 0, 0x03020000);
  CAN1.init_Filt(3, 0, 0x02120000);
  CAN1.init_Filt(4, 0, 0x02560000);
  CAN1.init_Filt(5, 0, 0x01160000);
  CAN1.setMode(MCP_NORMAL);
  pinMode(CAN1_INT, INPUT);

  // ── CAN2 init ─────────────────────────────────────────────────────────────
  if (CAN2.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN2 OK");
  } else {
    Serial.println("CAN2 FAIL");
  }
  CAN2.init_Mask(0, 0, 0x07FF0000);
  CAN2.init_Mask(1, 0, 0x07FF0000);
  CAN2.init_Filt(0, 0, 0x020C0000);
  CAN2.init_Filt(1, 0, 0x021C0000);
  CAN2.init_Filt(2, 0, 0x023C0000);
  CAN2.init_Filt(3, 0, 0x02320000);
  CAN2.init_Filt(4, 0, 0x03060000);
  CAN2.init_Filt(5, 0, 0x020C0000);
  CAN2.setMode(MCP_NORMAL);
  pinMode(CAN2_INT, INPUT);

  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // WiFi starts OFF — CAN task enables it when contactor closes
  WiFi.mode(WIFI_OFF);

  // ── Launch CAN task on core 0 ─────────────────────────────────────────────
  xTaskCreatePinnedToCore(
    taskCAN,        // function
    "taskCAN",      // name
    4096,           // stack bytes
    NULL,           // parameter
    2,              // priority (higher than WiFi for CAN latency)
    &taskCanHandle, // handle
    0               // core 0
  );

  // ── Launch WiFi/Web/ABRP task on core 1 ──────────────────────────────────
  xTaskCreatePinnedToCore(
    taskWifi,        // function
    "taskWifi",      // name
    10240,           // stack bytes (HTTP + TLS needs more room)
    NULL,            // parameter
    1,               // priority
    &taskWifiHandle, // handle
    1                // core 1
  );
}

// loop() is empty — everything runs inside FreeRTOS tasks
void loop() {
  vTaskDelay(portMAX_DELAY);
}
