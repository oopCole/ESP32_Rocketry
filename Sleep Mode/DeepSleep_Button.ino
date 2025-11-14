/*
  ESP32 Deep Sleep (EXT0 wake)
  - Button on GPIO33 (RTC IO), pulled down with 10k to GND, pressed to 3.3V
  - External LED on GPIO26

  https://youtube.com/shorts/UI89TH5mOxw?si=FiEh85cTQ_5rc1Op
*/

#include "driver/rtc_io.h"

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)
#define USE_EXT0_WAKEUP          1
#define WAKEUP_GPIO              GPIO_NUM_33         // button pin (RTC IO)
#define LED_PIN                  26                  // external LED pin

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup: EXT0 (RTC_IO)"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup: EXT1 (RTC_CNTL)"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup: TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup: TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup: ULP"); break;
    default:                        Serial.printf("Wakeup not from deep sleep: %d\n", wakeup_reason); break;
  }
}

void go_to_sleep() {
  // Configure wake source
#if USE_EXT0_WAKEUP
  // Wake when WAKEUP_GPIO is HIGH
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);

  // Internal pulls so the pin idles LOW during sleep
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
#else
  esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
#endif

  Serial.println("Going to deep sleep...");
  delay(50);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  ++bootCount;
  Serial.println("\nBoot number: " + String(bootCount));
  print_wakeup_reason();

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // keep off by default

  // Read the wake cause
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  // If we woke up because the button was pressed, turn the LED on and stay awake
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0
   || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("Button wake detected → turning LED ON.");
    digitalWrite(LED_PIN, HIGH);

    // If you want to go back to sleep after some time, uncomment the next lines:
    // delay(5000); // keep LED on for 5 seconds
    // digitalWrite(LED_PIN, LOW);
    // go_to_sleep();

  } else {
    // Cold boot or other wake cause: immediately arm wakeup and sleep
    // (LED stays off until a button wake happens)
    go_to_sleep();
  }
}

void loop() {
  // Not used
}
