/*
  WARNING: wakeup/sleep can affect servo
  =======
  RESET button (EN): MCU resets → setup() runs → servo goes to HOME (40°).
  BOOT button (GPIO 0): press while running → servo goes to RELEASE (90°), then deep sleep.
  Wake from sleep: GPIO 33. Deep sleep mode.
*/

#include <ModbusMaster.h>

// MAX485 to ESP32 wiring
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4   // MAX485 DE & RE tied to ESP32 GPIO4

// #define LED_DELAY_MS  1000   // LED standard delay

// Buttons: RESET (EN) = hardware reset → setup() runs → servo home
//          BOOT (GPIO 0) = press in loop → servo release
#define BOOT_BUTTON_PIN  0   // BOOT button on board (GPIO 0)
#define DEBOUNCE_MS      50

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << (GPIO))
#define USE_EXT0_WAKEUP          1
#define WAKEUP_GPIO              GPIO_NUM_33         // button pin (RTC IO)

// LED pins
// #define LED1_PIN  12
// #define LED2_PIN  13
// #define LED3_PIN  14

// Servo
#define SERVO_PIN   2
#define PWM_FREQ    333 
#define PWM_RES     16
#define SERVO_MOVE_MS  500   // time for servo to move
#define POST_SERVO_SLEEP_MS  3000  // delay before deep sleep 

// Modbus settings from manual 
static const uint8_t SLAVE_ID = 1;  // do not change or i will kms
static const uint32_t MODBUS_BAUD = 4800; // dont change this either 

// Servo pulse width at 0° and 180° (change these if your servo has different limits)
static const int PULSE_0_US   = 500;   // pulse width at 0°
static const int PULSE_180_US = 2500;  // pulse width at 180°

// Home and release positions in degrees (0–180). Servo goes here on reset vs boot button.
static const int ANGLE_MIN = 50;   // home (reset)
static const int ANGLE_MAX = 10;   // release (boot button)

// === sleep ===
void go_to_sleep() {
  // Configure wake source
#if USE_EXT0_WAKEUP
  // Wake when WAKEUP_GPIO is HIGH
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);

  // Internal pulls so the pin idles LOW during sleep (ESP32 3.x API)
  gpio_pullup_dis(WAKEUP_GPIO);
  gpio_pulldown_en(WAKEUP_GPIO);
#else
  esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
  gpio_pulldown_en(WAKEUP_GPIO);
  gpio_pullup_dis(WAKEUP_GPIO);
#endif

  // Serial.println("Going to deep sleep...");
  delay(50);
  esp_deep_sleep_start();
}

static uint32_t period_us() {
  return 1000000UL / PWM_FREQ;
}

static uint32_t usToDuty(uint32_t pulse_us) {
  uint32_t maxDuty = (1UL << PWM_RES) - 1UL;
  uint32_t period  = period_us();
  if (pulse_us > period) pulse_us = period;
  return (pulse_us * maxDuty) / period;
}

static void writeServoPulse(uint32_t pulse_us) {
  ledcWrite(SERVO_PIN, usToDuty(pulse_us));
}

// Map angle in degrees (0–180) to pulse width. ANGLE_MIN/ANGLE_MAX set home/release positions.
static void writeServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  long pulse = map(angle, 0, 180, PULSE_0_US, PULSE_180_US);
  writeServoPulse((uint32_t)pulse);
}

// static void allLedsOff() {
//   digitalWrite(LED1_PIN, LOW);
//   digitalWrite(LED2_PIN, LOW);
//   digitalWrite(LED3_PIN, LOW);
// }

// true when BOOT button is pressed (active LOW with internal pull-up)
static bool bootButtonPressed() {
  return digitalRead(BOOT_BUTTON_PIN) == LOW;
}

void setup() {
  Serial.begin(115200);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  
  // pinMode(LED1_PIN, OUTPUT);
  // pinMode(LED2_PIN, OUTPUT);
  // pinMode(LED3_PIN, OUTPUT);
  // allLedsOff();

  bool ok = ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES);
  if (!ok) {
    Serial.println("LEDC attach failed!");
    while (true);
  }
  // RESET button (or power-on): servo to home. Skip when waking from deep sleep.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    writeServoAngle(ANGLE_MIN);   // home 
  }
}

void loop() {
  // wait for BOOT button press (with debounce)
  if (!bootButtonPressed()) {
    delay(10);
    return;
  }
  delay(DEBOUNCE_MS);
  if (!bootButtonPressed()) return;

  // allLedsOff();
  // digitalWrite(LED1_PIN, HIGH);
  // delay(LED_DELAY_MS);

  // digitalWrite(LED2_PIN, HIGH);
  // delay(LED_DELAY_MS);

  // digitalWrite(LED3_PIN, HIGH);
  // delay(LED_DELAY_MS);

  // BOOT button pressed → servo to release (90°)
  writeServoAngle(ANGLE_MAX);   // release
  delay(SERVO_MOVE_MS);
  // writeServoAngle(40); 
  // delay(SERVO_MOVE_MS);

  // deep cleep
  delay(POST_SERVO_SLEEP_MS);
  go_to_sleep();
}
