/*
  auger, leadscrew, pump

  *** note that ESP32 GPIO 34/35 are input only and internal pullups can be wacky on some boards.
      if floating switch behavior, move limit pins to 25/26/27/32/33 etc
*/

#include <Arduino.h>
#include <ModbusMaster.h>

// MAX485 to ESP32 wiring
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4    // MAX485 DE & RE tied to ESP32 GPIO4

// Modbus settings from manual 
static const uint8_t SLAVE_ID = 1;  // do not change or i will kms
static const uint32_t MODBUS_BAUD = 4800; // dont change this either 

ModbusMaster node;

// RS485 direction control
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}
void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

void soilSensorInit() {
  Serial.begin(115200); // <<< IMPORTANT
  delay(300);

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // receive mode by default

  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

static bool soilSensorSampleOnce(float &ec_uScm, float &ph, float &n_mgkg) {
  // Read 3 registers starting at 0x0002:
  // 0x0002 = EC (uS/cm)
  // 0x0003 = pH (raw * 0.1)
  // 0x0004 = Nitrates (mg/kg)
  uint8_t result = node.readHoldingRegisters(0x0002, 3);

  if (result == node.ku8MBSuccess) {
    uint16_t ec_raw = node.getResponseBuffer(0);  // reg 0x0002
    uint16_t ph_raw = node.getResponseBuffer(1);  // reg 0x0003
    uint16_t n_raw  = node.getResponseBuffer(2);  // reg 0x0004

    ec_uScm = (float)ec_raw; 
    ph = ph_raw * 0.1f;       // pH scaled by 0.1 per manual
    n_mgkg = (float)n_raw;

    Serial.print(" EC: "); Serial.print(ec_uScm, 0); Serial.println(" uS/cm");
    Serial.print(" pH: "); Serial.println(ph, 1);
    Serial.print("  N: "); Serial.print(n_mgkg, 0); Serial.println("  mg/kg");
    Serial.println("===============");
    
    return true;
  } else {
    Serial.print("=== SENSOR FAILURE === Error code: 0x");
    Serial.println(result, HEX);
    return false;
  }
}

static bool runSoilSensorSequenceAvg_1min(float &ec_avg, float &ph_avg, float &n_avg) {
  const uint32_t duration_ms = 60000;
  const uint32_t interval_ms = 3000;

  uint32_t start = millis();
  uint32_t lastSample = 0;

  float ec_sum = 0.0f;
  float ph_sum = 0.0f;
  float n_sum  = 0.0f;
  uint32_t good = 0;

  while (millis() - start < duration_ms) {
    if (millis() - lastSample >= interval_ms) {
      lastSample = millis();

      float ec, ph, n;
      if (soilSensorSampleOnce(ec, ph, n)) {
        ec_sum += ec;
        ph_sum += ph;
        n_sum  += n;
        good++;
      }
    }
    delay(5);
  }

  if (good == 0) {
    return false;
  }

  ec_avg = ec_sum / (float)good;
  ph_avg = ph_sum / (float)good;
  n_avg  = n_sum  / (float)good;

  Serial.println("===== 1 MINUTE AVERAGE =====");
  Serial.print(" EC avg: "); Serial.print(ec_avg, 0); Serial.println(" uS/cm");
  Serial.print(" pH avg: "); Serial.println(ph_avg, 1);
  Serial.print("  N avg: "); Serial.print(n_avg, 0); Serial.println("  mg/kg");
  Serial.println("============================");

  return true;
}

// auger MOSFET gate HIGH = auger running
static const int PIN_AUGER_MOSFET = 26;

// leadscrew motor driver (DRV8263H) control pins 
// *** assuming DIR/PH and PWM/EN
static const int PIN_LEAD_DIR = 27;   
static const int PIN_LEAD_PWM = 25;   

// limit switches active LOW
static const int PIN_LIMIT_BOTTOM = 34; // bottom limit
static const int PIN_LIMIT_HOME   = 35; // top limit

// pump control HIGH = running water
static const int PIN_PUMP = 14;

// PWM settings for leadscrew motor
static const int LEAD_PWM_CH   = 0;
static const int LEAD_PWM_FREQ = 20000;
static const int LEAD_PWM_RES  = 8;     // 0-255
static const int LEAD_PWM_DUTY = 200;   // 0-255 (speed)

static const uint32_t T_DOWN_TIMEOUT_MS = 25000;
static const uint32_t T_UP_TIMEOUT_MS   = 25000;

static const uint32_t T_DEPOSIT_SPIN_MS = 2500; // shake out soil spin time
static const int DEPOSIT_REPEATS = 3;

static const uint32_t T_PUMP_RUN_MS = 5000; // pump ON for 5 sec

// Limit switches 
// *** assuming active LOW when pressed
static inline bool limitBottomHit() { return digitalRead(PIN_LIMIT_BOTTOM) == LOW; }
static inline bool limitHomeHit()   { return digitalRead(PIN_LIMIT_HOME)   == LOW; }

// Auger MOSFET 
// *** assuming HIGH = ON
static inline void augerSet(bool on) { digitalWrite(PIN_AUGER_MOSFET, on ? HIGH : LOW); }

// Pump 
// *** assuming HIGH = ON
static inline void pumpSet(bool on)  { digitalWrite(PIN_PUMP, on ? HIGH : LOW); }

static inline void leadscrewStop() { ledcWrite(LEAD_PWM_CH, 0); }

// If leadscrew moves in wrong direction, swap HIGH/LOW
// *** if using IN1/IN2 or another interface, change here
static inline void leadscrewDown() {
  digitalWrite(PIN_LEAD_DIR, HIGH);  
  ledcWrite(LEAD_PWM_CH, LEAD_PWM_DUTY);
}
static inline void leadscrewUp() {
  digitalWrite(PIN_LEAD_DIR, LOW);
  ledcWrite(LEAD_PWM_CH, LEAD_PWM_DUTY);
}

static inline void allOff() {
  pumpSet(false);
  augerSet(false);
  leadscrewStop();
}

enum BubbleResult { WORKING = 0, DONE, ERROR };

enum State {
  ST_SPIN_AND_LOWER = 0,
  ST_STOP_SPIN_AND_MOVE_UP,
  ST_SPIN_IN_PLACE_DEPOSIT,
  ST_PUMP_WATER,
  ST_SOIL_HOOK,
  ST_SLEEP,
  ST_ERROR
};

struct RunContext {
  State state;
  uint32_t stateStartMs;
  int depositCount;
};

static RunContext ctx;

static inline void goState(State s) {
  ctx.state = s;
  ctx.stateStartMs = millis();
}

static BubbleResult bubbleSpinAndLower(uint32_t startMs) {
  if (limitBottomHit()) {
    leadscrewStop();
    augerSet(false);
    return DONE;
  }

  augerSet(true);
  leadscrewDown();

  if (limitBottomHit()) {
    leadscrewStop();
    return DONE;
  }
  if (millis() - startMs > T_DOWN_TIMEOUT_MS) {
    allOff();
    return ERROR;
  }
  return WORKING;
}

static BubbleResult bubbleStopSpinAndMoveUp(uint32_t startMs) {
  if (limitHomeHit()) {
    leadscrewStop();
    augerSet(false);
    return DONE;
  }

  augerSet(false);
  leadscrewUp();

  if (limitHomeHit()) {
    leadscrewStop();
    return DONE;
  }
  if (millis() - startMs > T_UP_TIMEOUT_MS) {
    allOff();
    return ERROR;
  }
  return WORKING;
}

static BubbleResult bubbleSpinInPlaceDeposit(uint32_t startMs) {
  leadscrewStop();
  augerSet(true);

  if (millis() - startMs >= T_DEPOSIT_SPIN_MS) {
    augerSet(false);
    return DONE;
  }
  return WORKING;
}

static BubbleResult bubblePumpWater(uint32_t startMs) {
  pumpSet(true);

  if (millis() - startMs >= T_PUMP_RUN_MS) {
    pumpSet(false);
    return DONE;
  }
  return WORKING;
}

static BubbleResult bubbleSoilSensorHook() {
  float ec_avg = 0.0f;
  float ph_avg = 0.0f;
  float n_avg  = 0.0f;

  bool ok = runSoilSensorSequenceAvg_1min(ec_avg, ph_avg, n_avg);

  if (ok) {
    return DONE;
  }

  return ERROR;
}

// sleep mode (mission complete)
static void bubbleSleepNow() {
  allOff();
  delay(200);
  esp_deep_sleep_start(); 
}

static void smInit() {
  ctx.depositCount = 0;
  goState(ST_SPIN_AND_LOWER);
}

static void smLoop() {
  switch (ctx.state) {

    case ST_SPIN_AND_LOWER: {
      BubbleResult r = bubbleSpinAndLower(ctx.stateStartMs);
      if (r == DONE) goState(ST_STOP_SPIN_AND_MOVE_UP);
      else if (r == ERROR) goState(ST_ERROR);
      break;
    }

    case ST_STOP_SPIN_AND_MOVE_UP: {
      BubbleResult r = bubbleStopSpinAndMoveUp(ctx.stateStartMs);
      if (r == DONE) goState(ST_SPIN_IN_PLACE_DEPOSIT);
      else if (r == ERROR) goState(ST_ERROR);
      break;
    }

    // if deposit action == 3 
    case ST_SPIN_IN_PLACE_DEPOSIT: {
      BubbleResult r = bubbleSpinInPlaceDeposit(ctx.stateStartMs);
      if (r == DONE) {
        ctx.depositCount++;
        if (ctx.depositCount < DEPOSIT_REPEATS) {
          goState(ST_SPIN_AND_LOWER);
        } else {
          goState(ST_PUMP_WATER);
        }
      } else if (r == ERROR) {
        goState(ST_ERROR);
      }
      break;
    }

    case ST_PUMP_WATER: {
      BubbleResult r = bubblePumpWater(ctx.stateStartMs);
      if (r == DONE) goState(ST_SOIL_HOOK);
      else if (r == ERROR) goState(ST_ERROR);
      break;
    }

    case ST_SOIL_HOOK: {
      BubbleResult r = bubbleSoilSensorHook();
      if (r == DONE) goState(ST_SLEEP);
      else if (r == ERROR) goState(ST_ERROR);
      break;
    }

    case ST_SLEEP:
      bubbleSleepNow();
      break; 

    case ST_ERROR:
    default:
      allOff();
      delay(1000);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_AUGER_MOSFET, OUTPUT);
  pinMode(PIN_LEAD_DIR, OUTPUT);
  pinMode(PIN_LEAD_PWM, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);

  pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_LIMIT_HOME, INPUT_PULLUP);

  // PWM init for leadscrew
  ledcSetup(LEAD_PWM_CH, LEAD_PWM_FREQ, LEAD_PWM_RES);
  ledcAttachPin(PIN_LEAD_PWM, LEAD_PWM_CH);

  soilSensorInit();

  allOff();
  smInit();
}

void loop() {
  smLoop();
}
