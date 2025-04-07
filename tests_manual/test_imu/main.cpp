#include <Arduino.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Wire.h>

Adafruit_LSM6DS3TRC imu;

// === Test parameters ===
#define IMU_I2C_ADDR 0x6A
#define TEST_DELAY_MS 1000

void scanI2CBus() {
  Serial.println("🔍 Scanning I2C bus...");
  byte count = 0;
  for (byte addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("✅ Device found at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("⚠️ No I2C devices found.");
  }
}

void printAccelRange() {
  Serial.print("Accelerometer range: ");
  switch (imu.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G: Serial.println("±2G"); break;
    case LSM6DS_ACCEL_RANGE_4_G: Serial.println("±4G"); break;
    case LSM6DS_ACCEL_RANGE_8_G: Serial.println("±8G"); break;
    case LSM6DS_ACCEL_RANGE_16_G: Serial.println("±16G"); break;
  }
}

void printGyroRange() {
  Serial.print("Gyro range: ");
  switch (imu.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS: Serial.println("125 degrees/s"); break;
    case LSM6DS_GYRO_RANGE_250_DPS: Serial.println("250 degrees/s"); break;
    case LSM6DS_GYRO_RANGE_500_DPS: Serial.println("500 degrees/s"); break;
    case LSM6DS_GYRO_RANGE_1000_DPS: Serial.println("1000 degrees/s"); break;
    case LSM6DS_GYRO_RANGE_2000_DPS: Serial.println("2000 degrees/s"); break;
    default: Serial.println("Unknown or unsupported range");
  }
}

void printAccelRate() {
  Serial.print("Accelerometer data rate: ");
  switch (imu.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:   Serial.println("0 Hz"); break;
    case LSM6DS_RATE_12_5_HZ:    Serial.println("12.5 Hz"); break;
    case LSM6DS_RATE_26_HZ:      Serial.println("26 Hz"); break;
    case LSM6DS_RATE_52_HZ:      Serial.println("52 Hz"); break;
    case LSM6DS_RATE_104_HZ:     Serial.println("104 Hz"); break;
    case LSM6DS_RATE_208_HZ:     Serial.println("208 Hz"); break;
    case LSM6DS_RATE_416_HZ:     Serial.println("416 Hz"); break;
    case LSM6DS_RATE_833_HZ:     Serial.println("833 Hz"); break;
    case LSM6DS_RATE_1_66K_HZ:   Serial.println("1.66 kHz"); break;
    case LSM6DS_RATE_3_33K_HZ:   Serial.println("3.33 kHz"); break;
    case LSM6DS_RATE_6_66K_HZ:   Serial.println("6.66 kHz"); break;
    default:                     Serial.println("Unknown");
  }
}

void printGyroRate() {
  Serial.print("Gyroscope data rate: ");
  switch (imu.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:   Serial.println("0 Hz"); break;
    case LSM6DS_RATE_12_5_HZ:    Serial.println("12.5 Hz"); break;
    case LSM6DS_RATE_26_HZ:      Serial.println("26 Hz"); break;
    case LSM6DS_RATE_52_HZ:      Serial.println("52 Hz"); break;
    case LSM6DS_RATE_104_HZ:     Serial.println("104 Hz"); break;
    case LSM6DS_RATE_208_HZ:     Serial.println("208 Hz"); break;
    case LSM6DS_RATE_416_HZ:     Serial.println("416 Hz"); break;
    case LSM6DS_RATE_833_HZ:     Serial.println("833 Hz"); break;
    case LSM6DS_RATE_1_66K_HZ:   Serial.println("1.66 kHz"); break;
    case LSM6DS_RATE_3_33K_HZ:   Serial.println("3.33 kHz"); break;
    case LSM6DS_RATE_6_66K_HZ:   Serial.println("6.66 kHz"); break;
    default:                     Serial.println("Unknown");
  }
}

void printAccelGyroData(const sensors_event_t& accel, const sensors_event_t& gyro) {
  Serial.printf("Accel [m/s^2]: X=%.2f Y=%.2f Z=%.2f\n",
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  Serial.printf("Gyro [rad/s]:  X=%.2f Y=%.2f Z=%.2f\n",
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("[TEST START] IMU (LSM6DS33)");

  // Initialize IMU via I2C
  if (!imu.begin_I2C(IMU_I2C_ADDR)) {
    Serial.println("❌ IMU not detected at address 0x6A");
    Serial.println("[TEST RESULT] FAIL");
    return;
  }

  Serial.println("✅ IMU detected at address 0x6A");
  printAccelRange();
  printGyroRange();
  printAccelRate();
  printGyroRate();

  // Read one set of sensor data
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  Serial.printf("Temperature: %.2f °C\n", temp.temperature);
  printAccelGyroData(accel, gyro);

  // Basic sanity check
  bool valid = !(isnan(accel.acceleration.x) || isnan(gyro.gyro.x));
  if (valid) {
    Serial.println("[TEST RESULT] PASS");
  } else {
    Serial.println("[TEST RESULT] FAIL");
  }
}

void loop() {
  // No loop needed for single-shot test
}
