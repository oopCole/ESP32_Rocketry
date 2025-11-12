// ESP32 + MPL3115A2 + ICM-20948
// SDA = GPIO21, SCL = GPIO22

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <ICM_20948.h>   // SparkFun ICM-20948 library

// === Uncomment to output CSV instead of human-friendly text ===
// #define OUTPUT_CSV

Adafruit_MPL3115A2 mpl;
ICM_20948_I2C icm;

bool mpl_ok = false;
bool icm_ok = false;

void setup() {
  Serial.begin(9600);
  delay(300);

  // Use ESP32 default I2C pins (you can change if needed)
  Wire.begin(21, 22);

  Serial.println("\nStarting sensors...");

  // ---------- MPL3115A2 (0x60) ----------
  mpl_ok = mpl.begin();
  if (mpl_ok) {
    Serial.println("✅ MPL3115A2 detected at 0x60");
    // Optional: set your local sea-level pressure (hPa) to improve altitude
    // 1013.25 hPa is standard; convert to the Adafruit call in hPa
    mpl.setSeaPressure(1013.25);
  } else {
    Serial.println("❌ MPL3115A2 not detected (expect 0x60)");
  }

  // ---------- ICM-20948 (0x68 or 0x69) ----------
  icm.begin(Wire, 0); // 0 => 0x68
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 not found at 0x68, trying 0x69...");
    icm.begin(Wire, 1); // 1 => 0x69
  }
  icm_ok = (icm.status == ICM_20948_Stat_Ok);
  Serial.println(icm_ok ? "✅ ICM-20948 ready" : "❌ ICM-20948 not responding");

#ifndef OUTPUT_CSV
  Serial.println("\n------ RUNNING ------\n");
#else
  // CSV header
  Serial.println(
    "time_ms,"
    "mpl_alt_m,mpl_pres_Pa,mpl_temp_C,"
    "acc_x_mg,acc_y_mg,acc_z_mg,"
    "gyr_x_dps,gyr_y_dps,gyr_z_dps,"
    "mag_x_uT,mag_y_uT,mag_z_uT,"
    "icm_temp_C"
  );
#endif
}

void loop() {
#ifndef OUTPUT_CSV
  Serial.println("------ SENSOR READINGS ------");
#endif

  // Timestamp for CSV
  unsigned long t = millis();

  // ---------- Read MPL3115A2 ----------
  float alt_m = NAN, pres_Pa = NAN, mpl_temp_C = NAN;
  if (mpl_ok) {
    alt_m     = mpl.getAltitude();     // meters
    pres_Pa   = mpl.getPressure();     // Pascals
    mpl_temp_C= mpl.getTemperature();  // °C
  }

  // ---------- Read ICM-20948 ----------
  float acc_x= NAN, acc_y= NAN, acc_z= NAN; // mg
  float gyr_x= NAN, gyr_y= NAN, gyr_z= NAN; // dps
  float mag_x= NAN, mag_y= NAN, mag_z= NAN; // uT
  float icm_temp_C = NAN;

  if (icm_ok && icm.dataReady()) {
    icm.getAGMT(); // fills accel/gyro/mag/temp

    acc_x = icm.accX();  acc_y = icm.accY();  acc_z = icm.accZ();  // mg
    gyr_x = icm.gyrX();  gyr_y = icm.gyrY();  gyr_z = icm.gyrZ();  // dps
    mag_x = icm.magX();  mag_y = icm.magY();  mag_z = icm.magZ();  // uT
    icm_temp_C = icm.temp(); // °C
  }

#ifndef OUTPUT_CSV
  // ---------- Pretty, labeled output ----------
  if (mpl_ok) {
    Serial.print("MPL Altitude (m): ");   Serial.println(alt_m, 2);
    Serial.print("MPL Pressure (Pa): ");  Serial.println(pres_Pa, 2);
    Serial.print("MPL Temp (°C): ");      Serial.println(mpl_temp_C, 2);
  } else {
    Serial.println("MPL3115A2 not initialized.");
  }

  if (icm_ok) {
    Serial.println("ICM Accel (mg):");
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n", acc_x, acc_y, acc_z);

    Serial.println("ICM Gyro (dps):");
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n", gyr_x, gyr_y, gyr_z);

    Serial.println("ICM Mag (uT):");
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n", mag_x, mag_y, mag_z);

    Serial.print("ICM Temp (°C): ");
    Serial.println(icm_temp_C, 2);
  } else {
    Serial.println("ICM-20948 not initialized.");
  }

  Serial.println("------------------------------\n");
#else
  // ---------- CSV output (easy to log/plot) ----------
  // Use 'nan' where data isn't available
  Serial.printf(
    "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    t,
    alt_m, pres_Pa, mpl_temp_C,
    acc_x, acc_y, acc_z,
    gyr_x, gyr_y, gyr_z,
    mag_x, mag_y, mag_z,
    icm_temp_C
  );
#endif

  delay(1000);
}
