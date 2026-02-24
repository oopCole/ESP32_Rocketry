// Servo stuff
#define SERVO_PIN    4
#define PWM_FREQ     333
#define PWM_RES      16

static const int PULSE_MIN_US = 800;
static const int PULSE_MAX_US = 2200;
static const int ANGLE_MIN    = -60;
static const int ANGLE_MAX    =  60;

static const int ANGLE_INIT = 50; // home 
static const int ANGLE_GO   = 10; // release

uint32_t period_us() {
  return 1000000UL / PWM_FREQ;
}

uint32_t usToDuty(uint32_t pulse_us) {
  uint32_t maxDuty = (1UL << PWM_RES) - 1UL;
  uint32_t period  = period_us();
  if (pulse_us > period) pulse_us = period;
  return (pulse_us * maxDuty) / period;
}

void writeServoPulse(uint32_t pulse_us) {
  ledcWrite(SERVO_PIN, usToDuty(pulse_us));
}

void writeServoAngle(int angle) {
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
  long pulse = map(angle, ANGLE_MIN, ANGLE_MAX, PULSE_MIN_US, PULSE_MAX_US);
  writeServoPulse((uint32_t)pulse);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Attach PWM to servo pin
  bool ok = ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES);
  if (!ok) {
    Serial.println("LEDC attach failed!");
    while (true) {}
  }

  writeServoAngle(ANGLE_INIT); // home

  delay(3000); // 3 second timer === CHANGE ===

  writeServoAngle(ANGLE_GO); // release
}

void loop() {
  // servo holds position
  delay(100);
}
