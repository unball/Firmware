#ifndef IMU_H
#define IMU_H

#define IMU_DEBUG false

#define IMU_NUMBER                  0       //numero gravado no IMU
#define IMU_I2C_ADDRESS             0x68    // endereçõ de comunicação com IMU
#define IMU_ACCEL_START             0x3B    // R
#define IMU_GYRO_START              0x43    // R
#define IMU_TEMP_START              0x41    // R
#define PWR_MGMT_1                  0x6B    // R/W
#define MPU6050_WHO_AM_I            0x75    // R
#define IMU_ACELL_CONFIG            0x1C    // Configure the accelerometer scale
#define IMU_GYRO_CONFIG             0x1B    // Configure the gyroscope scale

#include <i2c_t3.h>
#include <math.h>

namespace Imu{

    typedef struct{
        double x;
        double y;
        double z;
    } axis;

    typedef struct{
        axis accel;
        axis gyro;
        double temp;
        double roll;
        double pitch;
    } imuAll;

    axis accel, gyro, accelOffset, gyroOffset;
    imuAll imuData;
    double accelScale[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    double gyroScale[4] = {131.0, 65.5, 32.8, 16.4};

    //zera o sleep bit do registrador de power management
    void imuStart(){
        uint8_t error;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(PWR_MGMT_1);     
        Wire.write(0);              //writes 0 to wake the imu
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.print("start: ");Serial.println(error);
    }

    // Configuring accel scale
    // 0 -> +/-2g; 1 -> +/-4g; 2 -> +/-8g; 3 -> +/-16g
    void imuAccelScale(uint8_t scale){
        uint8_t error;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_ACELL_CONFIG);
        Wire.write(scale<<3);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.print("accel config: ");Serial.println(error);
    }

    // Configuring gyro scale
    // 0 -> +/-250; 1 -> +/-500; 2 -> +/-1000; 3 -> +/-2000
    void imuGyroScale(uint8_t scale){
        uint8_t error;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(IMU_GYRO_CONFIG);
        Wire.write(scale<<3);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.print("gyro config: ");Serial.println(error);
    }

    //lê size bytes a partir do endereço address e salva em buffer
    void imuRegRead(uint8_t address, size_t size, int8_t *buffer){
        uint8_t error;
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(0x41);
        error = Wire.endTransmission();
        if(IMU_DEBUG)
            Serial.print("imuRegRead: ");Serial.println(error);

        Wire.requestFrom(address, size); 
        for(uint16_t i = 0; i < size; i++)
            buffer[i] = Wire.read();
    }

    //concatena size bytes, em pares, de from e salva em to
    void to16(int16_t *to, int8_t *from, uint8_t size){
        for(uint8_t i = 0; i < size; i++)
            to[i] = from[i*2]<<8 | from[(i*2)+1];
    }

    //faz a leitura do acelerômetro e calcula o valor em m/s usando accelScale[scale]
    void accelRead(int8_t scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_ACCEL_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        accel.x = (rawBuffer[0]*9.81)/accelScale[scale];
        accel.y = (rawBuffer[1]*9.81)/accelScale[scale];
        accel.z = (rawBuffer[2]*9.81)/accelScale[scale];
    }

    //faz a leitura do giroscópio e calcula o valor em graus/s usando gyroScale[scale]
    void gyroRead(int8_t scale){
        int8_t regBuffer[6];
        int16_t rawBuffer[3];
        imuRegRead(IMU_GYRO_START, 6, regBuffer);
        to16(rawBuffer, regBuffer, 3);
        gyro.x = rawBuffer[0]/gyroScale[scale];
        gyro.y = rawBuffer[1]/gyroScale[scale];
        gyro.z = rawBuffer[2]/gyroScale[scale];
    }

    //retorna temperatura lida em celcius
    double tempRead(){
        int8_t regBuffer[2];
        int16_t rawBuffer[1];
        double temp;
        imuRegRead(IMU_TEMP_START, 2, regBuffer);
        to16(rawBuffer, regBuffer, 1);
        temp = (rawBuffer[0]/340.0) + 36.53;
        return temp;
    }

    //filtro de leitura do acelerômetro e giroscópio
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

    //configura o IMU para leitura
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

    //chama a leitura do acelerômetro e giroscópio
    void imuRead(){
        accelRead(0);
        gyroRead(3);
        mediaMovel();
    }

}
#endif
