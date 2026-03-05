/*
  Combined soil sensors
  - STEMMA (I2C): moisture (0-1), temperature (C)
  - Primary (RS485 Modbus): EC (uS/cm), pH, NO3-N (mg/kg)
  Runs for 3 minutes, logging every 1 second.
*/

#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <ModbusMaster.h>

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

void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}
void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

// ----- Logging -----
const unsigned long RUN_DURATION_MS = 180000;  // 3 minutes
const unsigned long LOG_INTERVAL_MS = 1000;    // 1 second
unsigned long runStartMs;
unsigned long lastLogMs;

void setup() {
  Serial.begin(115200);
  delay(500);

  // STEMMA I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ss.begin(SEESAW_ADDR)) {
    Serial.println("STEMMA sensor not found");
    while (1);
  }
  Serial.println("STEMMA sensor OK");

  // Primary sensor RS485
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  Serial.println("Primary sensor (Modbus) OK");

  runStartMs = millis();
  lastLogMs = runStartMs;

  Serial.println("=== Starting 3-minute log (1 sec interval) ===");
  Serial.println("sec\tMoist\tTempC\tEC_uS\tpH\tN_mgkg");
  Serial.println("----------------------------------------------");
}

void loop() {
  unsigned long now = millis();

  // Stop after 3 minutes
  if (now - runStartMs >= RUN_DURATION_MS) {
    Serial.println("=== 3 minutes elapsed. Done. ===");
    while (1) {
      delay(1000);
    }
  }

  // Log every 1 second
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    unsigned long elapsedSec = (now - runStartMs) / 1000;

    // ----- Read STEMMA -----
    uint16_t raw = ss.touchRead(0);
    float moistureNorm = (raw - 200.0f) / 1800.0f;
    moistureNorm = constrain(moistureNorm, 0.0f, 1.0f);
    float tempC = ss.getTemp();

    // ----- Read Primary (EC, pH, N) -----
    float ec_uScm = 0;
    float ph = 0;
    float n_mgkg = 0;
    uint8_t result = node.readHoldingRegisters(0x0002, 3);
    if (result == node.ku8MBSuccess) {
      ec_uScm = (float)node.getResponseBuffer(0);
      ph = node.getResponseBuffer(1) * 0.1f;
      n_mgkg = (float)node.getResponseBuffer(2);
    }

    // One line per second: sec, moisture, tempC, EC, pH, N
    Serial.print(elapsedSec);
    Serial.print("\t");
    Serial.print(moistureNorm, 3);
    Serial.print("\t");
    Serial.print(tempC, 1);
    Serial.print("\t");
    Serial.print(ec_uScm, 0);
    Serial.print("\t");
    Serial.print(ph, 1);
    Serial.print("\t");
    Serial.println(n_mgkg, 0);
  }

  delay(50);  // small yield between checks
}
