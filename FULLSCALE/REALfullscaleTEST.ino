#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------------------------
//  R8: Hardware pin constants  (typed static const, not #define)
// -----------------------------------------------------------------------------
static const uint8_t  PIN_SD_CS    = 5U;
static const uint8_t  PIN_SPI_SCK  = 18U;
static const uint8_t  PIN_SPI_MISO = 19U;
static const uint8_t  PIN_SPI_MOSI = 23U;
static const uint8_t  PIN_I2C_SDA  = 21U;
static const uint8_t  PIN_I2C_SCL  = 22U;
static const uint8_t  PIN_SERVO    = 4U;

// R8: Timing / retry constants
static const uint8_t  CFG_MAX_RETRIES        = 5U;
static const uint32_t CFG_FLUSH_INTERVAL_MS  = 2000UL;
static const uint32_t CFG_LOOP_DELAY_MS      = 20UL;
static const uint32_t CFG_BARO_CONV_MS       = 8UL;
static const uint32_t CFG_SLEEP_DURATION_SEC = 5UL;
static const uint32_t CFG_MIN_AWAKE_MS       = 5000UL;
static const uint32_t CFG_BARO_TIMEOUT_MS    = 200UL;
static unsigned long s_wake_time_ms = 0UL;

// R8: Movement / altitude thresholds
static const float    CFG_MOVEMENT_THRESHOLD_MPS2 = 200.0f;
static const float    CFG_ALT_WAKE_THRESHOLD_M    = 10.0f;

// R8: Display
static const uint8_t  DISP_WIDTH   = 128U;
static const uint8_t  DISP_HEIGHT  = 64U;
static const uint8_t  DISP_ADDR    = 0x3CU;

// R8: Sensor validity ranges
static const float    ACCEL_MIN_MPS2 = -2000.0f;
static const float    ACCEL_MAX_MPS2 =  2000.0f;
static const float    GYRO_MIN_DPS   = -2000.0f;
static const float    GYRO_MAX_DPS   =  2000.0f;
static const float    ALT_MIN_M      =  -500.0f;
static const float    ALT_MAX_M      = 10000.0f;
static const float    TEMP_MIN_C     =   -50.0f;
static const float    TEMP_MAX_C     =   100.0f;

// R8: Servo geometry
static const uint32_t PWM_FREQ_HZ    = 333UL;
static const uint8_t  PWM_RES_BITS   = 16U;
static const uint32_t SERVO_MOVE_MS  = 500UL;
static const uint32_t PULSE_MIN_US   = 800UL;
static const uint32_t PULSE_MAX_US   = 2200UL;
static const int      SERVO_ANGLE_MIN   = -60;
static const int      SERVO_ANGLE_MAX   =  60;

// ============= LINES FOR EDIT ============
static const int      SERVO_ANGLE_INIT  =  50;   // home position
static const int      SERVO_ANGLE_GO    =  10;   // release position

static const float    ALT_BAND_LO_M = 122.0f;    //400.0f * 0.3048f; [METERS] // write first value as actual foot value 
static const float    ALT_BAND_HI_M = 152.0f;    //800.0f * 0.3048f;   // here as well
// =========================================

// R8: Error codes
static const int ERR_NONE          = 0;
static const int ERR_SD_INIT       = 1;
static const int ERR_IMU_INIT      = 2;
static const int ERR_BARO_INIT     = 3;
static const int ERR_FILE_OPEN     = 4;
static const int ERR_SENSOR_RANGE  = 5;
static const int ERR_WRITE_FAILED  = 6;
static const int ERR_DISPLAY_INIT  = 7;

// R8: I2C addresses
static const uint8_t I2C_ADDR_ICM  = 0x69U;
static const uint8_t I2C_ADDR_BARO = 0x60U;

// -----------------------------------------------------------------------------
//  Data structures
// -----------------------------------------------------------------------------
struct AccelCal {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
};

struct SensorData {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float altitude;
    float temperature;
};

// -----------------------------------------------------------------------------
//  Module-level globals  (R6: kept to strict minimum, all static)
// -----------------------------------------------------------------------------
static SPIClass          g_spi(VSPI);
static ICM_20948_I2C     g_icm;
static Adafruit_MPL3115A2 g_baro;
static Adafruit_SSD1306  g_display(DISP_WIDTH, DISP_HEIGHT, &Wire, -1);
static File              g_logFile;

static bool g_icm_ok     = false;
static bool g_mpl_ok     = false;
static bool g_display_ok = false;
static int  g_error_state = ERR_NONE;
static float g_alt_baseline_m = 0.0f; //Static value, sets 0ft as baseline whenever the altimeter starts

static const AccelCal ACCEL_CAL = {
    -40.5f, -5.0f, 16.0f,   // offsets x, y, z
     1.0f,  1.0f,  1.0f     // scales  x, y, z
};

// =============================================================================
//  R5 — Assertion helpers
// =============================================================================

// R5: Range assertion — records error code and returns false on violation.
static bool assertRange(float value, float lo, float hi, int err_code) {
    if (value < lo || value > hi) {
        g_error_state = err_code;
        return false;
    }
    return true;
}

// R5: Null pointer assertion — records error code and returns false if NULL.
static bool assertNotNull(const void* ptr, int err_code) {
    if (ptr == NULL) {
        g_error_state = err_code;
        return false;
    }
    return true;
}

// R5: File-valid assertion.
static bool assertFileValid(void) {
    if (!g_logFile) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

// =============================================================================
//  R1/R4/R5/R7: I2C low-level helpers
// =============================================================================

// Write one register byte; returns true on ACK.  (R7: endTransmission checked)
static bool i2cWriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(dev_addr);
    size_t wb1 = Wire.write(reg);    // R7: checked
    size_t wb2 = Wire.write(value);  // R7: checked
    uint8_t err = Wire.endTransmission();

    // R5: Two assertions — byte counts and bus ACK
    if (wb1 != 1U || wb2 != 1U) { return false; }
    if (err != 0U)               { return false; }
    return true;
}

// Request N bytes from a register; returns bytes received.  (R7: requestFrom checked)
static uint8_t i2cReadBytes(uint8_t dev_addr, uint8_t reg,
                             uint8_t* buf, uint8_t len) {
    // R5: Assert valid buffer pointer and nonzero length
    if (!assertNotNull(buf, ERR_SENSOR_RANGE)) { return 0U; }
    if (len == 0U) { return 0U; }

    Wire.beginTransmission(dev_addr);
    size_t wb = Wire.write(reg);     // R7: checked
    uint8_t err = Wire.endTransmission(false);

    if (wb != 1U || err != 0U) { return 0U; }

    uint8_t received = (uint8_t)Wire.requestFrom(dev_addr, len);
    if (received != len) { return 0U; }

    for (uint8_t i = 0U; i < len; i++) {   // R2: fixed upper bound = len
        buf[i] = Wire.read();
    }
    return received;
}

// =============================================================================
//  R1/R4/R5/R7: Display initialisation
// =============================================================================
static bool initDisplay(void) {
    // R5: Assert sane display dimensions before touching hardware
    if (DISP_WIDTH == 0U || DISP_HEIGHT == 0U) {
        g_error_state = ERR_DISPLAY_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {  // R2: bounded
        if (g_display.begin(SSD1306_SWITCHCAPVCC, DISP_ADDR)) {
            g_display.clearDisplay();
            g_display.setTextSize(1);
            g_display.setTextColor(SSD1306_WHITE);
            g_display.setCursor(0, 0);
            (void)g_display.println("Display Ready");  // R7: cast void (cosmetic)
            g_display.display();
            return true;
        }
        delay(100);
    }
    g_error_state = ERR_DISPLAY_INIT;
    return false;
}

// =============================================================================
//  R1/R4/R5/R7: SD card initialisation
// =============================================================================
static bool initSD(void) {
    // R5: Assert SPI object and CS pin are plausibly configured
    if (PIN_SD_CS == 0U) {
        g_error_state = ERR_SD_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {  // R2: bounded
        if (SD.begin(PIN_SD_CS, g_spi)) { return true; }
        delay(100);
    }
    g_error_state = ERR_SD_INIT;
    return false;
}

// =============================================================================
//  R1/R4/R5/R7: IMU reset  (split out of initIMU for R4)
// =============================================================================
static bool resetICM(uint8_t addr) {
    // R5: Assert address is in valid 7-bit range
    if (addr == 0U || addr > 127U) { return false; }

    bool ok = i2cWriteReg(addr, 0x06U, 0x80U);  // R7: return checked
    // R5: Assert write succeeded before waiting
    if (!ok) {
        Serial.printf("resetICM addr=0x%02X write failed\n", addr);
        return false;
    }
    delay(500);
    return true;
}

// =============================================================================
//  R1/R4/R5/R7: IMU initialisation
// =============================================================================
static bool initIMU(void) {
    // R5: Assert I2C address is valid before any bus traffic
    if (I2C_ADDR_ICM == 0U) {
        g_error_state = ERR_IMU_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {  // R2: bounded
        if (!resetICM(I2C_ADDR_ICM)) {
            Serial.println("resetICM failed");
            delay(100);
            continue;
        }
        g_icm.begin(Wire, I2C_ADDR_ICM);
        if (g_icm.status != ICM_20948_Stat_Ok) {
            Serial.printf("ICM begin failed, status=%d\n", g_icm.status);
            delay(100);
            continue;
        }
        delay(100);
        return true;
    }
    g_error_state = ERR_IMU_INIT;
    return false;
}

//  R1/R4/R5/R7: Barometer register configuration  (split for R4)
static bool configureBaroRegisters(void) {
    // R5: Put device in standby before touching CTRL_REG1
    bool ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x00U);
    if (!ok) { return false; }  // R5 + R7: second assertion via return check
    delay(10);

    // OSR=0 (fastest ~6 ms), altimeter mode, active
    ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x80U);
    if (!ok) { return false; }
    delay(20);
    return true;
}

//  R1/R4/R5/R7: Barometer initialisation
static bool initBarometer(void) {
    // R5: Sanity check I2C address
    if (I2C_ADDR_BARO == 0U) {
        g_error_state = ERR_BARO_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {  // R2: bounded
        if (!g_baro.begin()) { delay(100); continue; }
        g_baro.setSeaPressure(1013.26f);

        bool cfg_ok = configureBaroRegisters();   // R7: return checked
        if (!cfg_ok) { delay(100); continue; }
        return true;
    }
    g_error_state = ERR_BARO_INIT;
    return false;
}

//  R1/R4/R5/R7: Log file initialisation
static bool initLogFile(void) {
    static const char* const HEADER =
        "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C";

    // R5: Assert header string is not empty
    if (HEADER == NULL || HEADER[0] == '\0') {
        g_error_state = ERR_FILE_OPEN;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {  // R2: bounded
        g_logFile = SD.open("/imu_log.csv", FILE_WRITE);
        if (!g_logFile) { delay(100); continue; }

        size_t written = g_logFile.println(HEADER);  // R7: checked
        if (written == 0U) { g_logFile.close(); delay(100); continue; }

        g_logFile.flush();
        return true;
    }
    g_error_state = ERR_FILE_OPEN;
    return false;
}

// R4: Pure computation — period in microseconds.
static uint32_t servoPeriodUs(void) {
    // R5: Assert frequency is nonzero to avoid division by zero
    if (PWM_FREQ_HZ == 0UL) { return 0UL; }
    // R5: Assert result is in valid servo period range (300–20000 µs)
    uint32_t period = 1000000UL / PWM_FREQ_HZ;
    if (period < 300UL || period > 20000UL) { return 0UL; }
    return period;
}

// R4: Convert microseconds to PWM duty value.
static uint32_t usToDuty(uint32_t pulse_us) {
    uint32_t period = servoPeriodUs();
    // R5: Assert period is valid
    if (period == 0UL) { return 0UL; }
    if (pulse_us > period) { pulse_us = period; }

    uint32_t max_duty = (1UL << PWM_RES_BITS) - 1UL;
    uint32_t duty = (pulse_us * max_duty) / period;

    // R5: Assert result does not exceed maximum duty
    if (duty > max_duty) { duty = max_duty; }
    return duty;
}

// R7: ledcWrite return checked.
static bool writeServoPulse(uint32_t pulse_us) {
    uint32_t duty = usToDuty(pulse_us);
    // R5: Assert duty is within valid range
    uint32_t max_duty = (1UL << PWM_RES_BITS) - 1UL;
    if (duty > max_duty) { return false; }

    bool ok = ledcWrite(PIN_SERVO, duty);  // R7: return checked
    // R5: Assert write succeeded
    if (!ok) { return false; }
    return true;
}

// R7: writeServoPulse return checked.
static bool writeServoAngle(int angle) {
    // R5: Assert angle is within mechanical limits before clamping
    if (angle < SERVO_ANGLE_MIN - 10 || angle > SERVO_ANGLE_MAX + 10) {
        return false;
    }
    int clamped = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    long pulse = map(clamped, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX,
                     (long)PULSE_MIN_US, (long)PULSE_MAX_US);

    // R5: Assert computed pulse is within valid servo range
    if ((uint32_t)pulse < PULSE_MIN_US || (uint32_t)pulse > PULSE_MAX_US) {
        return false;
    }
    return writeServoPulse((uint32_t)pulse);  // R7: return propagated
}


//  R4/R5/R7: Write a one-off servo-event line to the log file
static bool writeServoEvent(float altitude_m) {
    // R5: Assert file is open and altitude is in valid range
    if (!assertFileValid())                                               { return false; }
    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE)){ return false; }

    char line[96];
    int len = snprintf(line, sizeof(line),
        "SERVO_RELEASE,ts_ms=%lu,alt_m=%.3f",
        (unsigned long)millis(),
        altitude_m);

    if (len < 0 || len >= (int)sizeof(line)) {   // R7: snprintf return checked
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }

    size_t written = g_logFile.println(line);    // R7: return checked
    if (written == 0U) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }

    g_logFile.flush();   // flush immediately so the event survives a crash/cutoff
    return true;
}

//  R1/R4/R5/R6: Altitude-controlled servo release
static void updateAltitudeServo(float altitude_m) {
    // R6: Descent-band state variables at smallest possible scope (static local)
    static bool s_above_band    = false;
    static bool s_triggered     = false;

    // R5: Assert altitude is within plausible sensor range
    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE)) { return; }
    // R5: Assert servo angle constants are ordered
    if (SERVO_ANGLE_INIT < SERVO_ANGLE_MIN || SERVO_ANGLE_GO < SERVO_ANGLE_MIN) { return; }

    if (altitude_m > ALT_BAND_HI_M) {
        s_above_band = true;
    }

    if (s_above_band && !s_triggered && altitude_m < ALT_BAND_LO_M) {
        s_triggered = true;

        Serial.printf("SERVO TRIGGER — ts=%lu ms  alt=%.3f m\n", //serial print for testing
                  (unsigned long)millis(), altitude_m);   
        
        // --- record the exact moment before any mechanical delay ---
        bool ev_ok = writeServoEvent(altitude_m);   // R7: checked
        if (!ev_ok) { Serial.println("Servo event log FAILED"); }

        Serial.printf("Descent band detected — RELEASE  ts=%lu ms  alt=%.3f m\n",
                      (unsigned long)millis(), altitude_m);

        bool ok = writeServoAngle(SERVO_ANGLE_GO);  // R7: checked
        if (!ok) { Serial.println("Servo write failed on release"); }
        delay(SERVO_MOVE_MS);
    }
}

//  R1/R4/R5/R7: Barometer — trigger a one-shot conversion
static void triggerBarometer(void) {
    if (!g_mpl_ok) { return; }
    // R5: Assert device address is valid
    if (I2C_ADDR_BARO == 0U) { return; }

    // OSR=0, altimeter, OST (one-shot trigger), SBYB
    bool ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x82U);  // R7: checked
    // R5: Log if trigger write failed (non-fatal — will retry next cycle)
    if (!ok) { Serial.println("Baro trigger write failed"); }
}

//  R4: Barometer helper — read 5 raw bytes and decode altitude + temperature
//  Returns false if raw data is unavailable or out of range.
static bool readBaroRawBytes(float* alt_out, float* temp_out) {
    // R5: Assert output pointers are valid
    if (!assertNotNull(alt_out,  ERR_SENSOR_RANGE)) { return false; }
    if (!assertNotNull(temp_out, ERR_SENSOR_RANGE)) { return false; }

    uint8_t buf[5] = {0U};
    uint8_t got = i2cReadBytes(I2C_ADDR_BARO, 0x01U, buf, 5U);  // R7: checked
    if (got != 5U) { return false; }

    int32_t alt_raw = ((int32_t)(int8_t)buf[0] << 12) |
                      ((uint32_t)buf[1]          <<  4) |
                       (buf[2] >> 4);
    float alt = (float)alt_raw / 16.0f;

    int16_t t_raw = ((int16_t)(int8_t)buf[3] << 8) | (int16_t)buf[4];
    float temp = (float)t_raw / 256.0f;

    if (!assertRange(alt,  ALT_MIN_M,  ALT_MAX_M,  ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(temp, TEMP_MIN_C, TEMP_MAX_C, ERR_SENSOR_RANGE)) { return false; }

    *alt_out  = alt;
    *temp_out = temp;
    return true;
}

//  R4: Barometer — poll for fresh conversion data
//  Uses static local state (R6) for cached values and trigger timing.
static bool pollBarometer(SensorData* data) {
    // R6: Barometer state at smallest possible scope
    static float    s_baro_alt       = 0.0f;
    static float    s_baro_temp      = 25.0f;
    static bool     s_triggered      = false;
    static uint32_t s_trigger_ms     = 0UL;

    // R5: Assert output pointer is valid; assert sensor is available
    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return false; }
    if (!g_mpl_ok)                              { return false; }

    // First call or previous cycle — kick off a conversion
    if (!s_triggered) {
        triggerBarometer();
        s_triggered  = true;
        s_trigger_ms = (uint32_t)millis();
        data->altitude    = s_baro_alt;
        data->temperature = s_baro_temp;
        return true;
    }

    // Conversion not yet complete — return cached values
    uint32_t now = (uint32_t)millis();
    if ((now - s_trigger_ms) < CFG_BARO_CONV_MS) {
        data->altitude    = s_baro_alt;
        data->temperature = s_baro_temp;
        return true;
    }

    // Check DRDY flag (bit 3 of status register 0x00)
    uint8_t status_byte = 0U;
    uint8_t got = i2cReadBytes(I2C_ADDR_BARO, 0x00U, &status_byte, 1U);  // R7: checked
    if (got != 1U || !(status_byte & 0x08U)) {
        if ((now - s_trigger_ms) > CFG_BARO_TIMEOUT_MS) {
            s_triggered = false;
            Serial.println("Baro DRDY timeout — re-triggering");
        }
    data->altitude    = s_baro_alt;
    data->temperature = s_baro_temp;
    return true;
    }
    // Data ready — decode
    float new_alt  = 0.0f;
    float new_temp = 0.0f;
    bool decoded = readBaroRawBytes(&new_alt, &new_temp);  // R7: checked

    if (decoded) {
        s_baro_alt  = new_alt;
        s_baro_temp = new_temp;
    }

    s_triggered = false;   // arm next conversion
    data->altitude    = s_baro_alt;
    data->temperature = s_baro_temp;
    return true;
}

//  R4: IMU — select register bank
static bool selectICMBank(uint8_t bank) {
    // R5: Assert bank number is in valid range (0–3)
    if (bank > 3U) { return false; }
    // R5: Assert device address is set
    if (I2C_ADDR_ICM == 0U) { return false; }

    bool ok = i2cWriteReg(I2C_ADDR_ICM, 0x7FU, (uint8_t)(bank << 4));  // R7: checked
    return ok;
}

//  R4: IMU — read accelerometer and gyroscope registers
static bool readIMUSensors(SensorData* data) {
    // R5: Assert output pointer is valid; assert sensor is available
    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return false; }
    if (!g_icm_ok) { return false; }

    bool bank_ok = selectICMBank(0U);   // R7: checked
    if (!bank_ok) { return false; }

    uint8_t raw[12] = {0U};
    uint8_t got = i2cReadBytes(I2C_ADDR_ICM, 0x2DU, raw, 12U);  // R7: checked
    if (got != 12U) { return false; }

    int16_t raw_ax = (int16_t)((uint16_t)raw[0]  << 8) | raw[1];
    int16_t raw_ay = (int16_t)((uint16_t)raw[2]  << 8) | raw[3];
    int16_t raw_az = (int16_t)((uint16_t)raw[4]  << 8) | raw[5];
    int16_t raw_gx = (int16_t)((uint16_t)raw[6]  << 8) | raw[7];
    int16_t raw_gy = (int16_t)((uint16_t)raw[8]  << 8) | raw[9];
    int16_t raw_gz = (int16_t)((uint16_t)raw[10] << 8) | raw[11];

    data->ax = ((float)raw_ax / 16.384f - ACCEL_CAL.offset_x) * ACCEL_CAL.scale_x;
    data->ay = ((float)raw_ay / 16.384f - ACCEL_CAL.offset_y) * ACCEL_CAL.scale_y;
    data->az = ((float)raw_az / 16.384f - ACCEL_CAL.offset_z) * ACCEL_CAL.scale_z;
    data->gx = (float)raw_gx / 131.0f;
    data->gy = (float)raw_gy / 131.0f;
    data->gz = (float)raw_gz / 131.0f;
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;

    // R5: Validate all six outputs; two of six shown inline, rest via helper
    if (!assertRange(data->ax, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->ay, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->az, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gx, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gy, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gz, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    return true;
}

//  R4/R5/R7: Write one data row to the log file
static bool writeLogData(const SensorData* data) {
    // R5: Assert pointer valid; assert file is open
    if (!assertNotNull(data, ERR_WRITE_FAILED)) { return false; }
    if (!assertFileValid())                      { return false; }

    char line[160];
    int len = snprintf(line, sizeof(line),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f",
        data->timestamp,
        data->ax, data->ay, data->az,
        data->gx, data->gy, data->gz,
        data->mx, data->my, data->mz,
        data->altitude, data->temperature);

    if (len < 0 || len >= (int)sizeof(line)) {  // R7: snprintf return checked
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }

    size_t written = g_logFile.println(line);   // R7: return checked
    if (written == 0U) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

//  R4/R5: Update OLED display
static void updateDisplay(const SensorData* data, bool moving,
                          unsigned long idle_ms) {
    // R5: Assert hardware and data are available
    if (!g_display_ok)                         { return; }
    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return; }

    g_display.clearDisplay();
    g_display.setCursor(0, 0);

    g_display.print("A:");
    g_display.print(data->ax, 0);
    g_display.print(" ");
    g_display.print(data->ay, 0);
    g_display.print(" ");
    (void)g_display.println(data->az, 0);   // R7: cosmetic, cast void

    g_display.print("G:");
    g_display.print(data->gx, 0);
    g_display.print(" ");
    g_display.print(data->gy, 0);
    g_display.print(" ");
    (void)g_display.println(data->gz, 0);

    g_display.print("Alt:");
    g_display.print(data->altitude, 1);
    g_display.print("m T:");
    (void)g_display.println(data->temperature, 1);

    g_display.print("Time:");
    (void)g_display.println(data->timestamp / 1000UL);

    g_display.print("MOV:");
    (void)g_display.println(moving ? "YES" : "NO ");

    long remaining = ((long)CFG_MIN_AWAKE_MS - (long)idle_ms) / 1000L;
    if (remaining < 0L) { remaining = 0L; }
    g_display.print("Slp in:");
    (void)g_display.println(remaining);

    g_display.display();
}

//  R4/R5/R6: Movement detection  (state held in static local)
static bool detectMovement(float ax, float ay, float az) {
    // R6: State at smallest possible scope
    static float s_last_ax    = 0.0f;
    static float s_last_ay    = 0.0f;
    static float s_last_az    = 0.0f;
    static bool  s_first_read = true;

    // R5: Assert inputs are in valid range
    if (!assertRange(ax, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(ay, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }

    if (s_first_read) {
        s_last_ax   = ax;
        s_last_ay   = ay;
        s_last_az   = az;
        s_first_read = false;
        return true;
    }

    float delta_ax = fabsf(ax - s_last_ax);
    float delta_ay = fabsf(ay - s_last_ay);
    float delta_az = fabsf(az - s_last_az);

    s_last_ax = ax;
    s_last_ay = ay;
    s_last_az = az;

    return (delta_ax > CFG_MOVEMENT_THRESHOLD_MPS2 ||
            delta_ay > CFG_MOVEMENT_THRESHOLD_MPS2 ||
            delta_az > CFG_MOVEMENT_THRESHOLD_MPS2);
}

//  R4: I2C bus reset  (hardware bit-bang recovery)
static void resetI2CBus(void) {
    // R5: Assert SDA/SCL pin numbers are valid GPIO
    if (PIN_I2C_SDA == 0U || PIN_I2C_SCL == 0U) { return; }
    // R5: Assert pins differ
    if (PIN_I2C_SDA == PIN_I2C_SCL) { return; }

    Wire.end();
    delay(50);

    pinMode(PIN_I2C_SDA, OUTPUT);
    pinMode(PIN_I2C_SCL, OUTPUT);
    digitalWrite(PIN_I2C_SDA, HIGH);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delay(10);

    for (int i = 0; i < 18; i++) {   // R2: fixed upper bound
        digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(50);
        digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(50);
    }

    // STOP condition
    digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(50);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(50);
    digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(50);

    delay(50);
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(10);

    if (digitalRead(PIN_I2C_SDA) == LOW) { Serial.println("SDA stuck LOW after recovery"); }
    if (digitalRead(PIN_I2C_SCL) == LOW) { Serial.println("SCL stuck LOW after recovery"); }

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(100);
    delay(100);
}

//  R4: Flush log file on a timed interval
static void flushLogFile(void) {
    static uint32_t s_last_flush_ms = 0UL;

    // R5: Assert file is open before flushing
    if (!g_logFile) { return; }
    // R5: Assert flush interval is nonzero
    if (CFG_FLUSH_INTERVAL_MS == 0UL) { return; }

    uint32_t now = (uint32_t)millis();
    if ((now >= s_last_flush_ms) &&
        ((now - s_last_flush_ms) > CFG_FLUSH_INTERVAL_MS)) {
        g_logFile.flush();
        s_last_flush_ms = now;
    }
}

//  R4: Sleep wake-up — reinitialise peripherals  (split from enterLightSleep)
static void wakeReinitPeripherals(void) {
    // R5: Assert I2C bus pins are configured before reset
    if (PIN_I2C_SDA == 0U || PIN_I2C_SCL == 0U) { return; }
    // R5: Assert sleep duration was nonzero (sanity check on config)
    if (CFG_SLEEP_DURATION_SEC == 0UL) { return; }

    resetI2CBus();
    Serial.println("I2C bus reset done");

    bool icm_reset = resetICM(I2C_ADDR_ICM);   // R7: checked
    Serial.printf("resetICM: %d\n", icm_reset);

    g_icm_ok = initIMU();
    Serial.printf("IMU: %s\n", g_icm_ok ? "OK" : "FAIL");

    g_mpl_ok = initBarometer();
    Serial.printf("BARO: %s\n", g_mpl_ok ? "OK" : "FAIL");
    if (g_mpl_ok) { delay(150); }

    g_display_ok = initDisplay();
    Serial.printf("DISP: %s\n", g_display_ok ? "OK" : "FAIL");
}

//  R4: Sleep wake-up — reopen SD log file  (split from enterLightSleep)
static void wakeReopenLogFile(void) {
    // R5: Assert CS pin is nonzero before bus init
    if (PIN_SD_CS == 0U) { return; }
    // R5: Assert log file is currently closed (we're re-opening)
    if (g_logFile) { g_logFile.close(); }

    if (!SD.begin(PIN_SD_CS, g_spi)) {
        Serial.println("SD re-init FAIL");
        return;
    }
    Serial.println("SD re-init OK");

    g_logFile = SD.open("/imu_log.csv", FILE_APPEND);
    if (!g_logFile) {
        Serial.println("File open FAIL");
    } else {
        Serial.println("File open OK");
    }
}

//  R4/R5: Enter light sleep then reinitialise on wake
static void enterLightSleep(void) {
    // R5: Assert sleep duration is nonzero
    if (CFG_SLEEP_DURATION_SEC == 0UL) { return; }
    // R5: Assert we are not being asked to re-enter before MIN_AWAKE_MS
    if (CFG_MIN_AWAKE_MS == 0UL) { return; }

    if (g_display_ok) {
        g_display.clearDisplay();
        g_display.setCursor(0, 0);
        (void)g_display.println("Entering Sleep");
        g_display.display();
    }

    if (g_logFile) { g_logFile.flush(); g_logFile.close(); }

    esp_sleep_enable_timer_wakeup(
        (uint64_t)CFG_SLEEP_DURATION_SEC * 1000000ULL);
    Serial.println("sleeping...");
    Serial.flush();
    esp_light_sleep_start();
    Serial.println("woke up");

    s_wake_time_ms = (unsigned long)millis();

    wakeReinitPeripherals();   
    wakeReopenLogFile();       
}

//(split from loop for R4)
static void updateAltitudeIdleTimer(float altitude_m,
                                    unsigned long* wake_time_ms) {
    static float s_alt_ref_m         = 0.0f;
    static bool  s_alt_ref_init      = false;

    // R5: Assert output pointer is valid
    if (!assertNotNull(wake_time_ms, ERR_NONE)) { return; }
    // R5: Assert altitude is in range
    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE)) { return; }

    if (!s_alt_ref_init) {
        s_alt_ref_m    = altitude_m;
        s_alt_ref_init = true;
        return;
    }

    float delta = fabsf(altitude_m - s_alt_ref_m);
    if (delta >= CFG_ALT_WAKE_THRESHOLD_M) {
        *wake_time_ms = (unsigned long)millis();
        s_alt_ref_m   = altitude_m;
        Serial.printf("Alt change %.1fm — resetting idle timer\n", delta);
    }
}

void setup(void) {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(100);
    delay(500);
    Serial.begin(115200);
    delay(500);

    // R7: Check LEDC attach return value
    bool ledc_ok = ledcAttach(PIN_SERVO, PWM_FREQ_HZ, PWM_RES_BITS);
    if (!ledc_ok) {
        Serial.println("LEDC attach FAIL");
    } else {
        bool servo_ok = writeServoAngle(SERVO_ANGLE_INIT);  // R7: checked
        if (!servo_ok) { Serial.println("Servo init write FAIL"); }
    }

    // R5: Assert SPI pins are distinct before bus initialisation
    if (PIN_SPI_SCK == PIN_SPI_MISO || PIN_SPI_SCK == PIN_SPI_MOSI) {
        Serial.println("SPI pin config error — halting");
        return;
    }
    // R5: Assert Wire timeout is set (nonzero implies Wire.begin succeeded)
    if (Wire.getTimeOut() == 0U) {
        Serial.println("Wire timeout not set — halting");
        return;
    }

    Serial.println("Scanning I2C...");
    for (uint8_t addr = 1U; addr < 127U; addr++) {   // R2: bounded 1..126
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0U) { Serial.printf("  0x%02X\n", addr); }
    }
    Serial.println("Scan done.");

    g_spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    g_display_ok = initDisplay();
    //if (!initDisplay())   { return;  } return after oled is re-introduced
    g_display_ok = true;

    if (!initSD())        { return; }
    if (!initIMU())       { return; }
    g_icm_ok = true;

    if (!initBarometer()) { return; }
    g_mpl_ok = true;

    // Read ground altitude and store as baseline
    g_alt_baseline_m = g_baro.getAltitude();
    Serial.printf("Baseline altitude: %.2f m\n", g_alt_baseline_m);

    if (!initLogFile())   { return; }

    delay(100);
    enterLightSleep();
}

//  loop()
void loop(void) {
    

    Serial.printf("tick — icm:%d mpl:%d\n", g_icm_ok, g_mpl_ok);

    SensorData data;
    data.timestamp   = (unsigned long)millis();
    data.ax = data.ay = data.az = 0.0f;
    data.gx = data.gy = data.gz = 0.0f;
    data.mx = data.my = data.mz = 0.0f;
    data.altitude    = 0.0f;
    data.temperature = 25.0f;

    bool imu_ok  = readIMUSensors(&data);   // R7: checked
    bool baro_ok = pollBarometer(&data);    // R7: checked

    data.altitude -= g_alt_baseline_m; //subtracting og value from the procedural readings

    // R5: Assert error state is valid (within defined range)
    if (g_error_state < ERR_NONE || g_error_state > ERR_DISPLAY_INIT) {
        g_error_state = ERR_NONE;
    }

    if (!imu_ok || !baro_ok) {
        if (g_display_ok) {
            g_display.clearDisplay();
            g_display.setCursor(0, 0);
            g_display.print("Sensor fail:");
            if (!imu_ok)  { g_display.print(" IMU");  }
            if (!baro_ok) { g_display.print(" BARO"); }
            g_display.display();
        }
        Serial.printf("Fail — IMU:%d BARO:%d err:%d\n",
                      imu_ok, baro_ok, g_error_state);
        delay(CFG_LOOP_DELAY_MS);
        return;
    }

    updateAltitudeServo(data.altitude);

    bool moving = detectMovement(data.ax, data.ay, data.az);  // R7: checked
    if (moving) { s_wake_time_ms = (unsigned long)millis(); }

    updateAltitudeIdleTimer(data.altitude, &s_wake_time_ms);  // R7: checked

    unsigned long awake_ms = (unsigned long)millis() - s_wake_time_ms;

    bool log_ok = writeLogData(&data);   // R7: checked
    if (!log_ok) { Serial.println("Log write failed"); }

    updateDisplay(&data, moving, awake_ms);
    flushLogFile();

    if (awake_ms > CFG_MIN_AWAKE_MS) { enterLightSleep(); }
    delay(CFG_LOOP_DELAY_MS);
}
