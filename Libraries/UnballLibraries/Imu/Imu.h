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

    union accel_t_gyro_union{
        struct{
            uint8_t x_accel_h;
            uint8_t x_accel_l;
            uint8_t y_accel_h;
            uint8_t y_accel_l;
            uint8_t z_accel_h;
            uint8_t z_accel_l;
            uint8_t t_h;
            uint8_t t_l;
            uint8_t x_gyro_h;
            uint8_t x_gyro_l;
            uint8_t y_gyro_h;
            uint8_t y_gyro_l;
            uint8_t z_gyro_h;
            uint8_t z_gyro_l;
        } reg;
        struct{
            int16_t x_accel;
            int16_t y_accel;
            int16_t z_accel;
            int16_t temperature;
            int16_t x_gyro;
            int16_t y_gyro;
            int16_t z_gyro;
        } value;
    };

    void Setup(){
        int error;
        uint8_t c;

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


    void mediaMovel(accel_t_gyro_union newData){
        double alpha = 0.9;
        values.x_accel = (alpha*values.x_accel) + (1-alpha)*newData.value.x_accel;
        values.y_accel = (alpha*values.y_accel) + (1-alpha)*newData.value.y_accel;
        values.z_accel = (alpha*values.z_accel) + (1-alpha)*newData.value.z_accel;
        values.x_gyro = (alpha*values.x_gyro) + (1-alpha)*newData.value.x_gyro;
        values.y_gyro = (alpha*values.y_gyro) + (1-alpha)*newData.value.y_gyro;
        values.z_gyro = (alpha*values.z_gyro) + (1-alpha)*newData.value.z_gyro;
        values.pitch = atan2(values.x_accel,
            sqrt(sq(values.z_accel)+sq(values.y_accel)));
        values.roll = atan2(values.y_accel,
            sqrt(sq(values.z_accel)+sq(values.x_accel)));
        return;
    }

    void imuRead(){
        accel_t_gyro_union accel_t_gyro;

        MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

        uint8_t swap;
        #define SWAP(x,y) swap = x; x = y; y = swap

        SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
        SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
        SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
        SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
        SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
        SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
        SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

        mediaMovel(accel_t_gyro);

        return;
    }

}
#endif
