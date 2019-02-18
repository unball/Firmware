#ifndef IMU_H
#define IMU_H

#define IMU_DEBUG false

#define IMU_I2C_ADDRESS              0x68    // addres to read from IMU
#define IMU_ACCEL_START              0x3B    // R
#define IMU_GYRO_START               0x43    // R
#define IMU_TEMP_START               0x41    // R
#define PWR_MGMT_1                   0x6B    // R/W
#define MPU6050_WHO_AM_I             0x75    // R
#define IMU_ACELL_CONFIG             0x1C    // Configure the accelerometer scale
#define IMU_GYRO_CONFIG              0x1B    // Configure the gyroscope scale

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68

#include <i2c_t3.h>
#include <math.h>

namespace Imu{

    typedef struct{
        double x;
        double y;
        double z;
    } imuAccel;

    typedef struct{
        double x;
        double y;
        double z;
    } imuGyro;

    typedef struct{
        imuAccel accel;
        imuGyro gyro;
        double temp;
        double roll;
        double pitch;
    } imuAll;

    imuAccel accel;
    imuGyro gyro;
    imuAll imuData;
    float accelScale[4] = {16384.0, 8192.0, 4096.0, 2048.0}
    float gyroScale[4] = {131.0, 65.5, 32.8, 16.4}

    void imuStart(){
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(PWR_MGMT_1);     
        Wire.write(0);              //writes 0 to wake the imu
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.printf("start: %d\n", error);
    }

    // Configuring accel scale
    // 0 -> +/-2g; 1 -> +/-4g; 2 -> +/-8g; 3 -> +/-16g
    void imuAccelScale(uint8_t scale){
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_ACELL_CONFIG);
        Wire.write(scale<<3);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.printf("accel config: %d\n", error);
    }

    // Configuring gyro scale
    // 0 -> +/-250; 1 -> +/-500; 2 -> +/-1000; 3 -> +/-2000
    void imuGyroScale(uint8_t scale){
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_GYRO_CONFIG);
        Wire.write(scale<<3);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.printf("gyro config: %d\n", error);
    }

    void imuRegRead(uint8_t address, size_t size, int8_t *buffer){
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(0x41);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.printf("imuRegRead: %d\n", error);

        Wire.requestFrom(address, size); 
        for(int i = 0; i < size; i++)
            buffer[i] = Wire.read();
    }

    void to16(int16_t *to, int8_t *from, int size){
        for(int i = 0; i < size; i++)
            to[i] = from[i*2]<<8 | from[(i*2)+1]
    }

    void accelRead(int scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_ACCEL_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        accel.x = (rawBuffer*9.81)[0]/accelScale[scale];
        accel.y = (rawBuffer*9.81)[1]/accelScale[scale];
        accel.z = (rawBuffer*9.81)[2]/accelScale[scale];
    }

    void gyroRead(int scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_GYRO_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        gyro.x = rawBuffer[0]/gyroScale[scale];
        gyro.y = rawBuffer[1]/gyroScale[scale];
        gyro.z = rawBuffer[2]/gyroScale[scale];
    }

    double tempRead(){
        int8_t regBuffer[2];
        int16_t rawBuffer[1];
        double temp;
        imuRegRead(IMU_TEMP_START, 2, regBuffer);
        to16(rawBuffer, regBuffer, 1);
        temp = (rawBuffer[0]/340.0) + 36.53;
        return temp;
    }

    void mediaMovel(){
        double alpha = 0.9;
        imuData.accel.x = (alpha*imuData.accel.x) + (1-alpha)*accel.x;
        imuData.accel.y = (alpha*imuData.accel.y) + (1-alpha)*accel.y;
        imuData.accel.z = (alpha*imuData.accel.z) + (1-alpha)*accel.z;
        imuData.gyro.x = (alpha*imuData.gyro.x) + (1-alpha)*gyro.x;
        imuData.gyro.y = (alpha*imuData.gyro.y) + (1-alpha)*gyro.y;
        imuData.gyro.z = (alpha*imuData.gyro.z) + (1-alpha)*gyro.z;
        /*imuData.pitch = atan2(imuData.accel.x,
            sqrt(sq(imuData.accel.z)+sq(imuData.accel.y)));
        imuData.roll = atan2(imuData.accel.y,
            sqrt(sq(imuData.accel.z)+sq(imuData.accel.x)));*/
    }

    void Setup(){

        // Initialize the 'Wire' class for the I2C-bus.
        Wire.begin();

        // default at power-up:
        //    Gyro at 250 degrees second
        //    Acceleration at 2g
        //    Clock source at internal 8MHz
        //    The device is in sleep mode.
        
        imuStart();
        imuAccelScale(0);
        imuGyroScale(3);
    }

    void imuRead(){
        accelRead(0);
        gyroRead(3);
        mediaMovel(accel_t_gyro);
    }

}
#endif
