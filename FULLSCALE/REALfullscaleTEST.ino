#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_INA3221.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuration constants
#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define WAKE_PIN  15
#define MAX_RETRIES 5
#define FLUSH_INTERVAL_MS 2000
#define LOOP_DELAY_MS 20
#define BARO_CONVERSION_MS 8   // OSR=0: ~6ms, give 8ms margin

// Sleep configuration
#define SLEEP_DURATION_SEC 5
#define MOVEMENT_THRESHOLD 200.0f  // m/s² acceleration threshold
#define MIN_AWAKE_MS 5000

// Altitude band
#define ALT_BAND_FT_LO  290.0f  // ignore this ugh
#define ALT_BAND_FT_HI  310.0f  // ignore this ugh
#define ALT_WAKE_THRESHOLD_M 10.0f

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Sensor value ranges for validation
#define ACCEL_MIN -2000.0f
#define ACCEL_MAX  2000.0f
#define GYRO_MIN  -2000.0f
#define GYRO_MAX   2000.0f
#define ALT_MIN   -500.0f
#define ALT_MAX    10000.0f
#define TEMP_MIN  -50.0f
#define TEMP_MAX   100.0f

// Servo stuff
#define SERVO_PIN      4
#define PWM_FREQ       333
#define PWM_RES        16
#define SERVO_MOVE_MS  500
#define PULSE_MIN_US   800  // 800, 500
#define PULSE_MAX_US   2200 // 2200, 2500

static const int ANGLE_MIN    = -60;
static const int ANGLE_MAX    =  60;

// ============= LINES FOR EDIT =================
static const int ANGLE_INIT = 50; // home
static const int ANGLE_GO   = 10; // release

#define ALT_DESCENT_BAND_LO_M  (700.0f * 0.3048f)   // ~213 m
#define ALT_DESCENT_BAND_HI_M  (721.0f * 0.3048f)   // ~219 m
// ==============================================

// Descent-band state: only release when we've been above band then descend below
static bool has_been_above_band = false;
static bool servo_triggered     = false;

// Error codes
#define ERROR_NONE 0
#define ERROR_SD_INIT 1
#define ERROR_IMU_INIT 2
#define ERROR_BARO_INIT 3
#define ERROR_FILE_OPEN 4
#define ERROR_SENSOR_RANGE 5
#define ERROR_WRITE_FAILED 6
#define ERROR_DISPLAY_INIT 7

// Accelerometer calibration
struct AccelCal {
  float offset_x;
  float offset_y;
  float offset_z;
  float scale_x;
  float scale_y;
  float scale_z;
} accel_cal = {-40.5f, -5.0f, 16.0f, 1.0f, 1.0f, 1.0f};

// Sensor data structure
struct SensorData {
  unsigned long timestamp;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float altitude;
  float temperature;
};

// Sleep state tracking
struct SleepState {
  float last_ax;
  float last_ay;
  float last_az;
  bool first_read;
} sleep_state = {0.0f, 0.0f, 0.0f, true};

// Global objects (minimized where possible)
SPIClass spi(VSPI);
ICM_20948_I2C myICM;
Adafruit_MPL3115A2 baro;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
File logFile;

bool icm_ok = false;
bool mpl_ok = false;
bool display_ok = false;
int error_state = ERROR_NONE;
static unsigned long wake_time_ms = 0;
static float baro_alt  = 0.0f;
static float baro_temp = 25.0f;
static bool  baro_triggered = false;
static unsigned long baro_trigger_ms = 0;

static float alt_ref_m = 0.0f;
static bool  alt_ref_initialized = false;

// Rule 5: Assertion for recovery
bool assertRange(float value, float min, float max, int errorCode) {
  if (value < min || value > max) {
    error_state = errorCode;
    return false;
  }
  return true;
}

// Rule 5: Assertion for file operations
bool assertFileValid(void) {
  if (!logFile) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }
  return true;
}

// Rule 2 & 4: Bounded display initialization with fixed retry limit
bool initDisplay(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Display Ready");
      display.display();
      return true;
    }
    delay(100);
  }
  error_state = ERROR_DISPLAY_INIT;
  return false;
}

// Rule 2 & 4: Bounded initialization with fixed retry limit
bool initSD(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (SD.begin(SD_CS, spi)) {
      return true;
    }
    delay(100);
  }
  error_state = ERROR_SD_INIT;
  return false;
}

// Rule 2 & 4: Bounded IMU initialization
bool initIMU(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (!resetICM(0x69)) {
      Serial.println("resetICM failed");
      delay(100);
      continue;
    }
    myICM.begin(Wire, 0x69);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.printf("myICM.begin failed, status=%d\n", myICM.status);
      delay(100);
      continue;
    }
    // startupMagnetometer() removed — causes aux I2C hang on second getAGMT
    delay(100);
    return true;
  }
  error_state = ERROR_IMU_INIT;
  return false;
}

// Rule 2 & 4: Bounded barometer initialization
bool initBarometer(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (!baro.begin()) { delay(100); continue; }

    baro.setSeaPressure(1013.26);

    // Must go to standby before changing CTRL_REG1
    Wire.beginTransmission(0x60);
    Wire.write(0x26);
    Wire.write(0x00);  // standby
    Wire.endTransmission();
    delay(10);

    // OSR=0 (fastest ~6ms), altimeter mode, active
    Wire.beginTransmission(0x60);
    Wire.write(0x26);
    Wire.write(0x81);  // 0b10000001
    Wire.endTransmission();
    delay(20);

    return true;
  }
  error_state = ERROR_BARO_INIT;
  return false;
}

// Rule 2 & 4: Bounded file opening
bool initLogFile(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    logFile = SD.open("/imu_log.csv", FILE_WRITE);
    if (logFile) {
      size_t written = logFile.println("timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C");
      if (written > 0) {
        logFile.flush();
        return true;
      }
      logFile.close();
    }
    delay(100);
  }
  error_state = ERROR_FILE_OPEN;
  return false;
}

// ===== Servo start =====
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

// call every loop with current altitude; triggers servo only on descent through band
void updateAltitudeServo(float altitude_m) {
  // Arm only when we have genuinely climbed above the band (ascent does NOT trigger)
  if (altitude_m > ALT_DESCENT_BAND_HI_M) {
    has_been_above_band = true;
  }

  // release only when we've descended through to below the low band (descent only)
  if (has_been_above_band && altitude_m < ALT_DESCENT_BAND_LO_M) {
    if (!servo_triggered) {
      servo_triggered = true;
      Serial.println("Descent band detected — RELEASE");
      writeServoAngle(ANGLE_GO);
      delay(SERVO_MOVE_MS);
    }
  }
}
//  ===== Servo end=====

void triggerBarometer(void) {
  if (!mpl_ok) return;
  Wire.beginTransmission(0x60);
  Wire.write(0x26);
  Wire.write(0x83);  // 0b10000011: ALT + OSR=0 + OST + SBYB
  Wire.endTransmission();
  baro_triggered = true;
  baro_trigger_ms = millis();
}

// Returns true if fresh data was read, false if still waiting
bool pollBarometer(SensorData* data) {
  if (!mpl_ok || data == NULL) return false;

  if (!baro_triggered) {
    data->altitude    = baro_alt;
    data->temperature = baro_temp;
    triggerBarometer();
    return true;
  }

  if ((millis() - baro_trigger_ms) < BARO_CONVERSION_MS) {
    data->altitude    = baro_alt;
    data->temperature = baro_temp;
    return true;
  }

  Wire.beginTransmission(0x60);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x60, (uint8_t)1);
  if (!Wire.available()) {
    data->altitude = baro_alt;
    data->temperature = baro_temp;
    return true;
  }
  uint8_t status = Wire.read();
  if (!(status & 0x08)) {
    data->altitude = baro_alt;
    data->temperature = baro_temp;
    return true;
  }

  Wire.beginTransmission(0x60);
  Wire.write(0x01);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)0x60, (uint8_t)5) != 5) {
    data->altitude = baro_alt;
    data->temperature = baro_temp;
    triggerBarometer();
    return true;
  }

  uint8_t p_msb = Wire.read();
  uint8_t p_csb = Wire.read();
  uint8_t p_lsb = Wire.read();
  uint8_t t_msb = Wire.read();
  uint8_t t_lsb = Wire.read();

  int32_t alt_raw = ((int32_t)(int8_t)p_msb << 12) |
                    ((uint32_t)p_csb << 4) |
                    (p_lsb >> 4);
  float alt = alt_raw / 16.0f;

  int16_t t_raw = ((int16_t)(int8_t)t_msb << 8) | t_lsb;
  float temp = t_raw / 256.0f;

  if (!assertRange(alt,  ALT_MIN, ALT_MAX,  ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(temp, TEMP_MIN, TEMP_MAX, ERROR_SENSOR_RANGE)) return false;

  baro_alt  = alt;
  baro_temp = temp;
  data->altitude    = baro_alt;
  data->temperature = baro_temp;

  triggerBarometer();
  return true;
}

void selectICMBank(uint8_t bank) {
  Wire.beginTransmission(0x69);
  Wire.write(0x7F);
  Wire.write(bank << 4);
  Wire.endTransmission();
}

bool readIMUSensors(SensorData* data) {
  if (data == NULL) { error_state = ERROR_SENSOR_RANGE; return false; }
  if (!icm_ok) return false;

  selectICMBank(0);

  Wire.beginTransmission(0x69);
  Wire.write(0x2D);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)0x69, (uint8_t)12) != 12) return false;

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  float ax_raw = raw_ax / 16.384f;
  float ay_raw = raw_ay / 16.384f;
  float az_raw = raw_az / 16.384f;

  data->ax = (ax_raw - accel_cal.offset_x) * accel_cal.scale_x;
  data->ay = (ay_raw - accel_cal.offset_y) * accel_cal.scale_y;
  data->az = (az_raw - accel_cal.offset_z) * accel_cal.scale_z;

  data->gx = raw_gx / 131.0f;
  data->gy = raw_gy / 131.0f;
  data->gz = raw_gz / 131.0f;

  data->mx = 0.0f;
  data->my = 0.0f;
  data->mz = 0.0f;

  if (!assertRange(data->ax, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->ay, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->az, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gx, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gy, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gz, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;

  return true;
}

bool readBarometer(SensorData* data) {
  if (data == NULL || !mpl_ok) {
    return false;
  }

  data->altitude    = 0.0f;
  data->temperature = 25.0f;

  unsigned long start = millis();
  data->altitude = baro.getAltitude();
  if (millis() - start > 200) {
    Serial.println("BARO altitude timeout");
    return false;
  }

  start = millis();
  data->temperature = baro.getTemperature();
  if (millis() - start > 200) {
    Serial.println("BARO temp timeout");
    return false;
  }

  return true;
}

bool writeLogData(const SensorData* data) {
  if (data == NULL) {
    return false;
  }

  if (!assertFileValid()) {
    return false;
  }

  char line[160];
  int len = snprintf(line, sizeof(line),
           "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f",
           data->timestamp, data->ax, data->ay, data->az,
           data->gx, data->gy, data->gz,
           data->mx, data->my, data->mz,
           data->altitude, data->temperature);

  if (len < 0 || len >= (int)sizeof(line)) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }

  size_t written = logFile.println(line);
  if (written == 0) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }

  return true;
}

void updateDisplay(const SensorData* data, bool moving, unsigned long idle_ms) {
  if (!display_ok || data == NULL) return;

  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("A:");
  display.print(data->ax, 0);
  display.print(" ");
  display.print(data->ay, 0);
  display.print(" ");
  display.println(data->az, 0);

  display.print("G:");
  display.print(data->gx, 0);
  display.print(" ");
  display.print(data->gy, 0);
  display.print(" ");
  display.println(data->gz, 0);

  display.print("Alt:");
  display.print(data->altitude, 1);
  display.print("m  T:");
  display.print(data->temperature, 1);
  display.println("C");

  display.print("Time: ");
  display.print(data->timestamp / 1000);
  display.println("s");

  display.print("MOV: ");
  display.println(moving ? "YES" : "NO ");

  long remaining = ((long)MIN_AWAKE_MS - (long)idle_ms) / 1000;
  if (remaining < 0) remaining = 0;
  display.print("Sleep in: ");
  display.print(remaining);
  display.println("s");

  display.display();
}

bool detectMovement(float ax, float ay, float az) {
  if (sleep_state.first_read) {
    sleep_state.last_ax = ax;
    sleep_state.last_ay = ay;
    sleep_state.last_az = az;
    sleep_state.first_read = false;
    return true;
  }

  float delta_ax = abs(ax - sleep_state.last_ax);
  float delta_ay = abs(ay - sleep_state.last_ay);
  float delta_az = abs(az - sleep_state.last_az);

  sleep_state.last_ax = ax;
  sleep_state.last_ay = ay;
  sleep_state.last_az = az;

  return (delta_ax > MOVEMENT_THRESHOLD ||
          delta_ay > MOVEMENT_THRESHOLD ||
          delta_az > MOVEMENT_THRESHOLD);
}

void enterLightSleep(void) {
  if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Entering Sleep");
    display.display();
  }

  if (logFile) { logFile.flush(); logFile.close(); }

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);
  Serial.println("lalelulelo");
  Serial.flush();
  esp_light_sleep_start();
  Serial.println("wakey-wakey");
  wake_time_ms = millis();

  resetI2CBus();
  Serial.println("I2C bus reset done");

  bool icm_reset = resetICM(0x69);
  Serial.printf("resetICM result: %d\n", icm_reset);

  icm_ok = initIMU();
  Serial.printf("IMU after wake: %s  (icm_ok=%d)\n", icm_ok ? "OK" : "FAIL", icm_ok);
  Serial.printf("IMU: %s\n", icm_ok ? "OK" : "FAIL");

  mpl_ok = initBarometer();
  Serial.printf("BARO after wake: %s  (mpl_ok=%d)\n", mpl_ok ? "OK" : "FAIL", mpl_ok);
  Serial.printf("BARO: %s\n", mpl_ok ? "OK" : "FAIL");
  if (mpl_ok) delay(150);

  display_ok = initDisplay();
  Serial.printf("DISP: %s\n", display_ok ? "OK" : "FAIL");

  if (!SD.begin(SD_CS, spi)) {
    Serial.println("SD re-init FAIL");
  } else {
    Serial.println("SD re-init OK");
    logFile = SD.open("/imu_log.csv", FILE_APPEND);
    if (!logFile) {
      Serial.println("File open FAIL");
    } else {
      Serial.println("File open OK");
    }
  }
}

void flushLogFile(void) {
  static unsigned long lastFlush = 0;
  if (!logFile) return;

  unsigned long currentTime = millis();

  if (currentTime >= lastFlush &&
      (currentTime - lastFlush) > FLUSH_INTERVAL_MS) {
    logFile.flush();
    lastFlush = currentTime;
  }
}

void resetI2CBus(void) {
  Wire.end();
  delay(50);

  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(21, HIGH);
  digitalWrite(22, HIGH);
  delay(10);

  for (int i = 0; i < 18; i++) {
    digitalWrite(22, HIGH); delayMicroseconds(50);
    digitalWrite(22, LOW);  delayMicroseconds(50);
  }
  digitalWrite(21, LOW);  delayMicroseconds(50);
  digitalWrite(22, HIGH); delayMicroseconds(50);
  digitalWrite(21, HIGH); delayMicroseconds(50);

  delay(50);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  delay(10);

  if (digitalRead(21) == LOW) {
    Serial.println("SDA still stuck low after recovery!");
  }
  if (digitalRead(22) == LOW) {
    Serial.println("SCL still stuck low after recovery!");
  }

  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  delay(100);
}

bool resetICM(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x06);
  Wire.write(0x80);
  uint8_t err = Wire.endTransmission();
  Serial.printf("resetICM addr=0x%02X endTransmission err=%d\n", addr, err);
  if (err != 0) return false;

  delay(500);
  return true;
}

void setup() {
  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  delay(500);
  Serial.begin(115200);
  delay(500);

  if (!ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("LEDC attach failed!");
  } else {
    writeServoAngle(ANGLE_INIT);   // home at startup
  }

  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
    }
  }
  Serial.println("Scan complete.");

  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  if (!initDisplay()) return;
  display_ok = true;

  if (!initSD()) return;
  if (!initIMU()) return;

  icm_ok = true;

  if (!initBarometer()) return;

  mpl_ok = true;

  if (!initLogFile()) return;

  delay(100);
  wake_time_ms = millis();
  enterLightSleep();
}

void loop() {
  Serial.printf("loop tick — icm_ok:%d mpl_ok:%d\n", icm_ok, mpl_ok);

  SensorData data = {0};
  data.timestamp = millis();
  Serial.println("A");

  bool imu_success = readIMUSensors(&data);
  Serial.println("A2");
  bool baro_success = pollBarometer(&data);
  Serial.println("B");

  if (!imu_success || !baro_success) {
    if (display_ok) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Sensor fail: ");
      display.print(imu_success  ? "" : "IMU ");
      display.print(baro_success ? "" : "BARO");
      display.display();
    }
    Serial.printf("Sensor fail — IMU:%d BARO:%d err:%d\n",
                  imu_success, baro_success, error_state);
    delay(LOOP_DELAY_MS);
    return;
  }

  Serial.println("sensors OK");
  updateAltitudeServo(data.altitude);

  bool movement_detected = detectMovement(data.ax, data.ay, data.az);
  if (movement_detected) {
    wake_time_ms = millis();
  }

  if (!alt_ref_initialized) {
    alt_ref_m = data.altitude;
    alt_ref_initialized = true;
  }
  float alt_delta = fabsf(data.altitude - alt_ref_m);
  if (alt_delta >= ALT_WAKE_THRESHOLD_M) {
    wake_time_ms = millis();
    alt_ref_m = data.altitude;
    Serial.printf("Alt change %.1fm — resetting idle timer\n", alt_delta);
  }

  unsigned long awake_ms = millis() - wake_time_ms;

  writeLogData(&data);
  updateDisplay(&data, movement_detected, awake_ms);
  flushLogFile();

  if (awake_ms > MIN_AWAKE_MS) enterLightSleep();
  delay(LOOP_DELAY_MS);
}
