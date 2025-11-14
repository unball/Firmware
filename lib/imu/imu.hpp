#ifndef IMU_HPP
#define IMU_HPP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "../../include/pins.h"

#define IMU_I2C_ADDR 0x6B

namespace IMU {

    extern Adafruit_LSM6DS3TRC imu;

    void setup();
    float get_w(); // Angular velocity around Z axis in rad/s (raw from gyro)
    
    // Complementary filter for angular velocity
    float get_w_filtered(float w_encoders, float alpha = 0.98);

}

#endif
