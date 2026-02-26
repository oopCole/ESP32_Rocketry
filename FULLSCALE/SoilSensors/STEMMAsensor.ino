#include <Wire.h>
#include "Adafruit_seesaw.h"
// turn on time stamps

Adafruit_seesaw ss;

const uint8_t SEESAW_ADDR = 0x36;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Starting soil sensor...");

  if (!ss.begin(SEESAW_ADDR)) {

    Serial.println("Sensor not found");
    while (1);
  }

  Serial.println("Sensor found");
}

void loop() {
  uint16_t raw = ss.touchRead(0);           // 200 dry to 2000 wet
  float moistureNorm = (raw - 200.0f) / 1800.0f;
  moistureNorm = constrain(moistureNorm, 0.0f, 1.0f);

  float tempC = ss.getTemp();

  // Serial.print("Raw Moisture: ");
  // Serial.print(raw);
  Serial.print("  Moisture: "); 
  Serial.println(moistureNorm, 3);  // normalized 0.0 to 1.0 value

  Serial.print("    Temp C: ");
  Serial.println(tempC);  // Celcius

  delay(3000);
}