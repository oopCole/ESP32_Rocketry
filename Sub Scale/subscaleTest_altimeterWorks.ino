#include <Wire.h>
#include <Adafruit_MPL115A2.h>

Adafruit_MPL115A2 mpl;

void setup() {
  Serial.begin(115200);
  delay(500);

  // ESP32 default I2C pins
  // SDA = GPIO21, SCL = GPIO22
  Wire.begin(21, 22);

  Serial.println("\nESP32 w/ MPL115A2 Altimeter");

  if (!mpl.begin()) {
    Serial.println("MPL115A2 not detected");
    while (1) { delay(100); }
  }

  Serial.println("MPL115A2 ready");
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

void loop() {
  unsigned long t = millis();

  float pressure_kPa = mpl.getPressure();     
  float temperature_C = mpl.getTemperature(); 

  // Convert pressure to altitude (meters)
  // Standard atmosphere model
  float altitude_m =
    44330.0 * (1.0 - pow(pressure_kPa / 101.325, 0.1903));

  float t_s = millis() / 1000.0;

  // CSV output
  Serial.printf(
    "[Time(s): %.2f], [Pressure(kPa): %.2f], [Altitude(m): %.2f], [temp(°C): %.2f]\n",
    t_s, pressure_kPa, altitude_m, temperature_C
  );

  delay(3000); // time between readings
}
