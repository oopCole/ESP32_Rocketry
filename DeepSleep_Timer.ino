/*
  ESP32 Deep Sleep (Timer wake → button active when awake)
  - Button on GPIO33 (external 10k pulldown to GND, press ties to 3.3V)
  - External LED on GPIO26
  - Sleeps immediately on cold boot, wakes after 10 seconds by timer
  - While asleep: button does nothing
  - After wake: pressing the button turns the LED ON

  https://youtube.com/shorts/Df5EWCeMeLw?si=qilXFPKsgHH53Ag3
*/

#include "driver/rtc_io.h"
#include "esp_sleep.h"

#define WAKE_TIMER_SECONDS       10
#define BUTTON_GPIO              33
#define LED_PIN                  26

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("<AWAKE> Wakeup: EXT0 (RTC_IO)"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("<AWAKE> Wakeup: EXT1 (RTC_CNTL)"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("<AWAKE>"); break; // Wakeup: TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("<AWAKE> Wakeup: TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("<AWAKE> Wakeup: ULP"); break;
    default:                        Serial.printf("<ASLEEP>\n", wakeup_reason); break; // Wakeup not from deep sleep: %d\n", wakeup_reason); break;
  }
}

void go_to_sleep_timer(uint32_t seconds) {
  Serial.printf("Arming timer wake for %u seconds, going to deep sleep...\n", seconds);
  // Only a timer wake — button will NOT wake or light anything while sleeping
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL); // 10 seconds
  delay(50);
  esp_deep_sleep_start();
}

bool wait_for_button_press(uint32_t debounce_ms = 25) {
  // simple press detector (HIGH = pressed), with debounce
  if (digitalRead(BUTTON_GPIO) == HIGH) {
    delay(debounce_ms);
    if (digitalRead(BUTTON_GPIO) == HIGH) {
      // wait until release to avoid repeated triggers if desired
      while (digitalRead(BUTTON_GPIO) == HIGH) { delay(1); }
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  ++bootCount;
  //Serial.println();
  //Serial.println("==== ESP32 Timer-Wake then Button-Active ====");
  //Serial.println("Boot #: " + String(bootCount));
  print_wakeup_reason();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_GPIO, INPUT);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) {
    // Cold boot or other wake cause → immediately sleep and let the TIMER wake up in 10 s
    go_to_sleep_timer(WAKE_TIMER_SECONDS);
  }

  // If we reach here, we woke from the TIMER.
  //Serial.println("after timer. Button is now active; press it to turn LED ON.");
}

void loop() {
  // After timer wake, enable normal button behavior
  if (wait_for_button_press()) {
    Serial.println("Button pressed → LED ON");
    digitalWrite(LED_PIN, HIGH);
  }

  // Optional: add an inactivity timeout to go back to sleep.
  // e.g., track millis() and if no press for N seconds, call go_to_sleep_timer(WAKE_TIMER_SECONDS).

  delay(5);
}
