#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA 5
#define I2C_SCL 5
#define IMU_ADDR 0x6A

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🧭 IMU test started.");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(IMU_ADDR);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("✅ IMU found at 0x6A.");
  } else {
    Serial.println("❌ IMU not found. Check wiring.");
  }
}

void loop() {
  delay(1000);
}