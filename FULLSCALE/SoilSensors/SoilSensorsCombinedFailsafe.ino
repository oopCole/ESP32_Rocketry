/*
  Combined soil sensors + failsafes
  - STEMMA (I2C): moisture (0-1), temperature (C)
  - Primary (RS485 Modbus): EC (uS/cm), pH, NO3-N (mg/kg)
  Logs every 1 second until power-off or reset.

  Failsafes:
  (1) Modbus result codes + NA for failed primary reads
  (2) ESP32 reset reason + RTC boot counter
  (3) Task watchdog — resets if loop hangs
  (4) Post-read sanity checks on primary + STEMMA temperature

  "DataQ" - Data Quality
  OK - Modbus read succeeded, EC/pH/N look in range, STEMMA temperature in range.
  NA_MB - Modbus failed that second — EC/pH/N are printed as NA; check the Modbus column (TO, CRC, etc.).
  RANGE_BAD - Modbus succeeded but EC/pH/N failed the plausibility bounds (possible bad frame/wiring).
  STEMMA_TEMP - Modbus and primary ranges OK, but STEMMA temperature was outside −40…85 °C.
*/

#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <ModbusMaster.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// ----- STEMMA (I2C) -----
Adafruit_seesaw ss;
const uint8_t SEESAW_ADDR = 0x36;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ----- Primary soil sensor (RS485 Modbus) -----
#define RX2_PIN    16
#define TX2_PIN    17
#define RE_DE_PIN  4
static const uint8_t SLAVE_ID = 1;
static const uint32_t MODBUS_BAUD = 4800;
ModbusMaster node;

// ----- Watchdog -----
static const uint32_t WDT_TIMEOUT_SEC = 30;

// ----- Boot diagnostics (RTC RAM survives deep sleep; increments on most resets) -----
RTC_DATA_ATTR uint32_t rtcBootCount = 0;

void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}

void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

// ----- Logging -----
const unsigned long LOG_INTERVAL_MS = 1000;  // 1 second
unsigned long runStartMs;
unsigned long lastLogMs;

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

static void printBootDiagnostics() {
  rtcBootCount++;
  Serial.print("Reset reason: ");
  Serial.print(resetReasonStr(esp_reset_reason()));
  Serial.print(" | RTC boot count: ");
  Serial.println(rtcBootCount);
}

static bool initWatchdog() {
  // Arduino-ESP32 3.3.x / IDF 5+: esp_task_wdt_init takes esp_task_wdt_config_t* (not timeout+panic).
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_SEC * 1000u;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;

  esp_err_t e = esp_task_wdt_init(&cfg);
  if (e == ESP_ERR_INVALID_STATE) {
    // TWDT already running (some core builds init it) — just subscribe this task.
    e = esp_task_wdt_add(NULL);
    if (e != ESP_OK) {
      Serial.print("esp_task_wdt_add failed: ");
      Serial.println((int)e);
      return false;
    }
    return true;
  }
  if (e != ESP_OK) {
    Serial.print("esp_task_wdt_init failed: ");
    Serial.println((int)e);
    return false;
  }
  e = esp_task_wdt_add(NULL);
  if (e != ESP_OK) {
    Serial.print("esp_task_wdt_add failed: ");
    Serial.println((int)e);
    return false;
  }
  return true;
}

static const char* modbusResultTag(const ModbusMaster& m, uint8_t r) {
  if (r == m.ku8MBSuccess) return "OK";
  if (r == m.ku8MBResponseTimedOut) return "TO";
  if (r == m.ku8MBInvalidCRC) return "CRC";
  if (r == m.ku8MBInvalidSlaveID) return "BAD_SLAVE";
  if (r == m.ku8MBInvalidFunction) return "BAD_FN";
  if (r == m.ku8MBIllegalFunction) return "EX_ILLEGAL_FN";
  if (r == m.ku8MBIllegalDataAddress) return "EX_BAD_ADDR";
  if (r == m.ku8MBIllegalDataValue) return "EX_BAD_VAL";
  if (r == m.ku8MBSlaveDeviceFailure) return "EX_SLAVE_FAIL";
  static char buf[6];
  snprintf(buf, sizeof(buf), "0x%02X", r);
  return buf;
}

// Generous bounds — tighten if you know your probe’s spec
static bool primaryValuesInRange(float ec_uScm, float ph, float n_mgkg) {
  if (ph < 0.0f || ph > 14.0f) return false;
  if (ec_uScm < 0.0f || ec_uScm > 200000.0f) return false;
  if (n_mgkg < 0.0f || n_mgkg > 100000.0f) return false;
  return true;
}

static bool stemmaTempInRange(float tempC) {
  return tempC >= -40.0f && tempC <= 85.0f;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  printBootDiagnostics();

  if (!initWatchdog()) {
    Serial.println("Watchdog not enabled; continuing without WDT.");
  }

  // STEMMA I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ss.begin(SEESAW_ADDR)) {
    Serial.println("STEMMA sensor not found");
    while (1) {
      esp_task_wdt_reset();
      delay(500);
    }
  }
  Serial.println("STEMMA sensor OK");

  // Primary sensor RS485
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  delay(750);  // many Modbus devices need settle time after power/baud start
  Serial.println("Primary sensor (Modbus) link initialized");

  runStartMs = millis();
  lastLogMs = runStartMs;

  Serial.println("=== Starting continuous log (1 sec interval) ===");
  Serial.println("sec\tMoist\tTempC\tEC_uS\tpH\tN_mgkg\tModbus\tDataQ");
  Serial.println("----------------------------------------------------------------");
}

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Log every 1 second
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    unsigned long elapsedSec = (now - runStartMs) / 1000;

    // ----- Read STEMMA -----
    uint16_t raw = ss.touchRead(0);
    float moistureNorm = (raw - 200.0f) / 1800.0f;
    moistureNorm = constrain(moistureNorm, 0.0f, 1.0f);
    float tempC = ss.getTemp();
    bool stTempOk = stemmaTempInRange(tempC);

    // ----- Read Primary (EC, pH, N) -----
    float ec_uScm = 0;
    float ph = 0;
    float n_mgkg = 0;
    uint8_t mbResult = node.readHoldingRegisters(0x0002, 3);
    bool mbOk = (mbResult == node.ku8MBSuccess);
    if (mbOk) {
      ec_uScm = (float)node.getResponseBuffer(0);
      ph = node.getResponseBuffer(1) * 0.1f;
      n_mgkg = (float)node.getResponseBuffer(2);
    }

    bool primRangeOk = mbOk && primaryValuesInRange(ec_uScm, ph, n_mgkg);

    const char* dataq;
    if (!mbOk) {
      dataq = "NA_MB";
    } else if (!primRangeOk) {
      dataq = "RANGE_BAD";
    } else if (!stTempOk) {
      dataq = "STEMMA_TEMP";
    } else {
      dataq = "OK";
    }

    Serial.print(elapsedSec);
    Serial.print("\t");
    Serial.print(moistureNorm, 3);
    Serial.print("\t");
    Serial.print(tempC, 1);
    Serial.print("\t");
    if (mbOk) {
      Serial.print(ec_uScm, 0);
      Serial.print("\t");
      Serial.print(ph, 1);
      Serial.print("\t");
      Serial.print(n_mgkg, 0);
    } else {
      Serial.print("NA\tNA\tNA");
    }
    Serial.print("\t");
    Serial.print(modbusResultTag(node, mbResult));
    Serial.print("\t");
    Serial.println(dataq);
  }

  delay(50);
}
