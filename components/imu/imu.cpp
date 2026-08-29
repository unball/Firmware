#include "imu.hpp"

#include "esp_err.h"
#include "i2cdev.h"
#include "mpu6050.h"
#include <string.h>

static mpu6050_dev_t mpu_dev;

esp_err_t setup() {
  ESP_ERROR_CHECK(i2cdev_init()); // Initialize I²C subsystem

  memset(&mpu_dev, 0, sizeof(mpu6050_dev_t)); // Clean device descriptor

  // Configure device I²C interface
  ESP_ERROR_CHECK(
      mpu6050_init_desc(&mpu_dev, I2C_ADDR, I2C_PORT, SDA_PIN, SCL_PIN));

  // Configure internal clock
  ESP_ERROR_CHECK(mpu6050_set_clock_source(&mpu_dev, MPU6050_CLOCK_PLL_X));

  // Configure digital low-pass filter
  ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(
      &mpu_dev,
      MPU6050_DLPF_4)); // Accel. BW = 21Hz, Gyro. BW = 20Hz

  // Set value ranges
  ESP_ERROR_CHECK(
      mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_8));
  ESP_ERROR_CHECK(
      mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_500));

  // Wake up the device
  ESP_ERROR_CHECK(mpu6050_set_sleep_enabled(&mpu_dev, false));

  return ESP_OK;
}

esp_err_t get_motion(mpu6050_acceleration_t *accel, mpu6050_rotation_t *gyro) {
  return mpu6050_get_motion(&mpu_dev, accel, gyro);
}