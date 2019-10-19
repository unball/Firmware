#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <stdint-gcc.h>
#include <cstdio>
#include <math.h>  
#include <Wire.h>

#if TEENSY_DEBUG
#define IMU_DEBUG true
#else
#define IMU_DEBUG false
#endif

#define IMU_NUMBER                  0           //numero gravado no IMU
#define IMU_I2C_ADDRESS             0x68        // endereço de comunicação com IMU
#define IMU_ACCEL_START             0x3B        // R
#define IMU_TEMP_START              0x41        // R
#define IMU_GYRO_START              0x43        // R
#define PWR_MGMT_1                  0x6B        // R/W
#define MPU6050_WHO_AM_I            0x75        // R
#define IMU_ACELL_CONFIG            0x1C        // Configure the accelerometer scale
#define IMU_GYRO_CONFIG             0x1B        // Configure the gyroscope scale
#define IMU_FIFO_EN                 0x23

namespace Imu{
    typedef struct{
        double x;
        double y;
        double z;
    } axis;
    
    typedef struct{
        double lin;
        double ang;
    } vel;

    typedef struct{
        axis accel;
        axis gyro;
        vel velocidades;
        double temp;
        double roll;
        double pitch;
    } imuAll;

    void imuStart();
    void imuAccelScale(uint8_t);
    void imuGyroScale(uint8_t);
    void imuRegRead(uint8_t, size_t, int8_t*);
    void to16(int16_t*, int8_t*, uint8_t);
    void accelRead(int8_t);
    void gyroRead(int8_t);
    double tempRead();
    void mediaMovel();
    void Setup();
    imuAll imuRead();
    double deltaT();
    double linearVel(double bias);
}
#endif