/*
  primary soil sensor — JXBS-3001-TR style 7-in-1 on RS485 Modbus RTU (FC 0x03)
  reads only EC, pH, and soil nitrogen (N) from the vendor register map

  copied from SoilSensorW; original left unchanged for older 3-register probe

  https://youtube.com/shorts/boy_2yaQp2w?si=t9RISODULN1ZBLDq
*/

#include <ModbusMaster.h>

// MAX485 to ESP32 wiring
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4    // MAX485 DE & RE tied to ESP32 GPIO4

// modbus: factory default 9600 8N1, address 1 (2400/4800 if reconfigured)
static const uint8_t SLAVE_ID = 1;
static const uint32_t MODBUS_BAUD = 9600;

// holding register addresses (differs from older EC/pH/nitrate-only probe at 0x0002)
static const uint16_t REG_PH = 0x0006;
static const uint16_t REG_EC = 0x0015;
static const uint16_t REG_N  = 0x001E;

ModbusMaster node;

// rs485 direction control
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}
void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

void setup() {
  Serial.begin(115200); // <<< IMPORTANT
  delay(300);

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // receive mode by default

  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t r;

  r = node.readHoldingRegisters(REG_EC, 1);
  if (r != node.ku8MBSuccess) {
    Serial.print("=== SENSOR FAILURE === EC read Error code: 0x");
    Serial.println(r, HEX);
    delay(1000);
    return;
  }
  float ec_uScm = (float)node.getResponseBuffer(0);

  r = node.readHoldingRegisters(REG_PH, 1);
  if (r != node.ku8MBSuccess) {
    Serial.print("=== SENSOR FAILURE === pH read Error code: 0x");
    Serial.println(r, HEX);
    delay(1000);
    return;
  }
  float ph = node.getResponseBuffer(0) * 0.01f;

  r = node.readHoldingRegisters(REG_N, 1);
  if (r != node.ku8MBSuccess) {
    Serial.print("=== SENSOR FAILURE === N read Error code: 0x");
    Serial.println(r, HEX);
    delay(1000);
    return;
  }
  float n_mgkg = (float)node.getResponseBuffer(0);

  Serial.print(" EC: "); Serial.print(ec_uScm, 0); Serial.println(" uS/cm");
  Serial.print(" pH: "); Serial.println(ph, 2);
  Serial.print("  N: "); Serial.print(n_mgkg, 0); Serial.println(" mg/kg");
  Serial.println("===============");

  delay(1000);
}
