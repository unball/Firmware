#ifndef IMU_HPP
#define IMU_HPP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>

namespace IMU {

    extern Adafruit_LSM6DS3TRC imu;

    void setup();
    float get_w(); // Angular velocity around Z axis in rad/s

}

#endif
