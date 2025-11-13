#include <SPI.h>
#include <SD.h>

#define SD_CS 4   // SD Shield 3.0 uses D4 for chip select

void setup() {
  Serial.begin(115200);
  while (!Serial) { }  // just in case

  Serial.println("Testing SD card...");

  // CS pin as output, default HIGH
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(10);

  // Simple begin is fine; ESP32 core will use default SPI
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    return;
  }

  Serial.println("SD init succeeded!");

  File testFile = SD.open("/test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Hello SD!");
    testFile.close();
    Serial.println("File written successfully.");
  } else {
    Serial.println("Failed to open file!");
  }
}

void loop() {}
