/** Blinky for an external LED on ESP32
 * 
https://youtube.com/shorts/w4ZnrqSd9DY?si=8wSbwiHTCI8sRUrm
**/

// Choose the pin you wired the LED to:
const int LED_PIN = 0;   // Use GPIO 0 (or whatever pin you connected to)

void setup() {
  pinMode(LED_PIN, OUTPUT);  // set pin as output
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // LED ON
  delay(1000);                  // wait 1 second
  digitalWrite(LED_PIN, LOW);   // LED OFF
  delay(1000);                  // wait 1 second
}
