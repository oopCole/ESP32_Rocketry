
#include <ModbusMaster.h>

// MAX485 to ESP32 wiring
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4    // MAX485 DE & RE tied to ESP32 GPIO4

// Modbus settings from manual 
static const uint8_t SLAVE_ID = 1;  // do not change or i will kms
static const uint32_t MODBUS_BAUD = 4800; // dont change this either 

ModbusMaster node;

// RS485 direction control
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
  // Read 3 registers starting at 0x0002:
  // 0x0002 = EC (uS/cm)
  // 0x0003 = pH (raw * 0.1)
  // 0x0004 = Nitrogen (mg/kg)
  uint8_t result = node.readHoldingRegisters(0x0002, 3);

  if (result == node.ku8MBSuccess) {
    uint16_t ec_raw = node.getResponseBuffer(0);  // reg 0x0002
    uint16_t ph_raw = node.getResponseBuffer(1);  // reg 0x0003
    uint16_t n_raw  = node.getResponseBuffer(2);  // reg 0x0004

    float ec_uScm = (float)ec_raw; 
    float ph = ph_raw * 0.1f;       // pH scaled by 0.1 per manual
    float n_mgkg = (float)n_raw;

    Serial.print(" EC: "); Serial.print(ec_uScm, 0); Serial.println(" uS/cm");
    Serial.print(" pH: "); Serial.println(ph, 1);
    Serial.print("  N: "); Serial.print(n_mgkg, 0); Serial.println("  mg/kg");
    Serial.println("===============");
    
  } else {
    Serial.print("=== SENSOR FAILURE === Error code: 0x");
    Serial.println(result, HEX);
  }

  delay(3000);
}
