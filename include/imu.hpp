#ifndef IMU_HPP
#define IMU_HP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "pins.h"

namespace IMU{

    extern Adafruit_MPU6050 mpu;

    void setup();
    float get_theta_speed();
}

#endif