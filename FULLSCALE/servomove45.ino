/*
  WARNING: wakeup/sleep can affect servo
  =======
  Boot: LEDs light red, yellow, green. 
        Servo activates (home: 40°, release: 90°)
        Soil sensor
        Deep cleep mode
    EN: Restarts
*/

#include <ModbusMaster.h>

// MAX485 to ESP32 wiring
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4   // MAX485 DE & RE tied to ESP32 GPIO4

#define LED_DELAY_MS  1000   // LED standard delay

// button GPIO
#define BUTTON_PIN   0
#define DEBOUNCE_MS 50

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)
#define USE_EXT0_WAKEUP          1
#define WAKEUP_GPIO              GPIO_NUM_33         // button pin (RTC IO)

// LED pins
#define LED1_PIN  12
#define LED2_PIN  13
#define LED3_PIN  14

// Servo
#define SERVO_PIN   2
#define PWM_FREQ    333 
#define PWM_RES     16
#define SERVO_MOVE_MS  500   // time for servo to move 45 degrees
#define POST_SERVO_SLEEP_MS  3000  // delay before deep sleep 

// Modbus settings from manual 
static const uint8_t SLAVE_ID = 1;  // do not change or i will kms
static const uint32_t MODBUS_BAUD = 4800; // dont change this either 

static const int PULSE_MIN_US = 800;
static const int PULSE_MAX_US = 2200;
static const int ANGLE_MIN    = -60;
static const int ANGLE_MAX    =  90;   // home 40°, release 90°

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

static void writeServoAngle(int angle) {
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
  long pulse = map(angle, ANGLE_MIN, ANGLE_MAX, PULSE_MIN_US, PULSE_MAX_US);
  writeServoPulse(pulse);
}

static void allLedsOff() {
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
}

// true when button is pressed (active LOW with internal pull-up)
static bool buttonPressed() {
  return digitalRead(BUTTON_PIN) == LOW;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  allLedsOff();

  bool ok = ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES);
  if (!ok) {
    Serial.println("LEDC attach failed!");
    while (true);
  }
  // only send on cold boot, skip when waking from sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    writeServoAngle(40); // home
  }
}

void loop() {
  // wait for button press (with debounce)
  if (!buttonPressed()) {
    delay(10);
    return;
  }
  delay(DEBOUNCE_MS);
  if (!buttonPressed()) return;

  allLedsOff();
  digitalWrite(LED1_PIN, HIGH);
  delay(LED_DELAY_MS);

  digitalWrite(LED2_PIN, HIGH);
  delay(LED_DELAY_MS);

  digitalWrite(LED3_PIN, HIGH);
  delay(LED_DELAY_MS);

  // home 40°, release 90°
  writeServoAngle(90);   // release
  delay(SERVO_MOVE_MS);
  // writeServoAngle(40); 
  // delay(SERVO_MOVE_MS);

  // deep cleep
  delay(POST_SERVO_SLEEP_MS);
  go_to_sleep();
}
