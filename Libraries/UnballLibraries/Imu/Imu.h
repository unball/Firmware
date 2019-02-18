#ifndef IMU_H
#define IMU_H

#define IMU_DEBUG false

#define MPU6050_ACCEL_XOUT_H        0x3B    // R
#define MPU6050_PWR_MGMT_1          0x6B    // R/W
#define MPU6050_WHO_AM_I            0x75    // R
#define MPU6050_ACELL_CONFIG        0x1C    // Configure the accelerometer scale
#define MPU6050_GYRO_CONFIG         0x1B    // Configure the gyroscope scale

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68

#include <Wire.h>
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

    volatile accel values;

    

    int MPU6050_read(int start, uint8_t *buffer, int size){
        int i, n;

        Wire.beginTransmission(MPU6050_I2C_ADDRESS);
        n = Wire.write(start);
        if (n != 1)
            return (-10);

        n = Wire.endTransmission(false);    // hold the I2C-bus
        if (n != 0)
            return (n);

        // Third parameter is true: relase I2C-bus after data is read.
        Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
        i = 0;
        while(Wire.available() && i<size){
            buffer[i++]=Wire.read();
        }
        if ( i != size)
            return (-11);

        return (0);  // return : no error
    }


    // --------------------------------------------------------
    // MPU6050_write
    //
    // This is a common function to write multiple bytes to an I2C device.
    //
    // If only a single register is written,
    // use the function MPU_6050_write_reg().
    //
    // Parameters:
    //   start : Start address, use a define for the register
    //   pData : A pointer to the data to write.
    //   size  : The number of bytes to write.
    //
    // If only a single register is written, a pointer
    // to the data has to be used, and the size is
    // a single byte:
    //   int data = 0;        // the data to write
    //   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
    //
    int MPU6050_write(int start, const uint8_t *pData, int size){
        int n, error;
        Wire.beginTransmission(MPU6050_I2C_ADDRESS);
        n = Wire.write(start);        // write the start address
        if (n != 1)
            return (-20);

        n = Wire.write(pData, size);  // write data bytes
        if (n != size)
            return (-21);

        error = Wire.endTransmission(true); // release the I2C-bus
        if (error != 0)
            return (error);

        return (0);         // return : no error
    }

    // --------------------------------------------------------
    // MPU6050_write_reg
    //
    // An extra function to write a single register.
    // It is just a wrapper around the MPU_6050_write()
    // function, and it is only a convenient function
    // to make it easier to write a single register.
    //
    int MPU6050_write_reg(int reg, uint8_t data){
        int error;
        error = MPU6050_write(reg, &data, 1);
        return (error);
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
    //

        error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
        if(IMU_DEBUG){
            Serial.print(F("WHO_AM_I : "));
            Serial.print(c,HEX);
            Serial.print(F(", error = "));
            Serial.println(error,DEC);
        }
    // According to the datasheet, the 'sleep' bit
    // should read a '1'.
    // That bit has to be cleared, since the sensor
    // is in sleep mode at power-up.
        error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
        if(IMU_DEBUG){
            Serial.print(F("PWR_MGMT_1 : "));
            Serial.print(c,HEX);
            Serial.print(F(", error = "));
            Serial.println(error,DEC);
        }

    // Clear the 'sleep' bit to start the sensor.
        MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

    // Configuring accel scale
    // 0 -> +/-2g; 1 -> +/-4g; 2 -> +/-8g; 3 -> +/-16g
        MPU6050_write_reg (MPU6050_ACELL_CONFIG, 0);

    // Configuring gyro scale
    // 0 -> +/-250; 1 -> +/-500; 2 -> +/-1000; 3 -> +/-2000
        MPU6050_write_reg (MPU6050_GYRO_CONFIG, (0x02)<<3); //for some reason needs the "<<3"
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

/*
    void loop(){
        int error;
        accel_t_gyro_union accel_t_gyro;

    // Read the raw values.
    // Read 14 bytes at once,
    // containing acceleration, temperature and gyro.
    // With the default settings of the MPU-6050,
    // there is no filter enabled, and the values
    // are not very stable.
        error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
        if(IMU_DEBUG){
            Serial.print(F("Read accel, temp and gyro, error = "));
            Serial.println(error,DEC);
        }

    // Swap all high and low bytes.
    // After this, the registers values are swapped,
    // so the structure name like x_accel_l does no
    // longer contain the lower byte.
        uint8_t swap;
        #define SWAP(x,y) swap = x; x = y; y = swap

        SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
        SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
        SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
        SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
        SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
        SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
        SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


    // Print the raw acceleration values

        double pitch = atan2(accel_t_gyro.value.x_accel,
            sqrt(sq(accel_t_gyro.value.z_accel)+sq(accel_t_gyro.value.y_accel)));
    //    double roll = atan2(accel_t_gyro.value.y_accel,
    //        sqrt(sq(accel_t_gyro.value.z_accel)+sq(accel_t_gyro.value.x_accel)));
        double pitchDg = pitch*180/PI;
        if(IMU_DEBUG){
            Serial.print("pitch: ");
            Serial.println(pitchDg);
            Serial.print(F("accel x,y,z: \t"));
            Serial.print(accel_t_gyro.value.x_accel, DEC);
            Serial.print(F(", "));
        }
        double t = double(cos(pitch)*2*9.807*accel_t_gyro.value.x_accel/16384.0);
        if(IMU_DEBUG){
            Serial.print(t);
            Serial.print(F(", \t"));
            Serial.print(accel_t_gyro.value.y_accel, DEC);
            Serial.print(F(", "));
        }
        t = double(2*9.807*accel_t_gyro.value.y_accel/16384.0);
        if(IMU_DEBUG){
            Serial.print(F(", "));
            Serial.print(t);
            Serial.print(F(", \t"));
            Serial.print(accel_t_gyro.value.z_accel, DEC);
            Serial.print(F(", "));
        }
        t = double(cos(pitch)*2*9.807*accel_t_gyro.value.z_accel/16384.0);
        if(IMU_DEBUG)
            Serial.println(t);

    // The temperature sensor is -40 to +85 degrees Celsius.
    // It is a signed integer.
    // According to the datasheet:
    //   340 per degrees Celsius, -512 at 35 degrees.
    // At 0 degrees: -512 - (340 * 35) = -12412



    // Print the raw gyro values.
        if(IMU_DEBUG){
            Serial.print(F("gyro x,y,z : "));
            Serial.print(accel_t_gyro.value.x_gyro, DEC);
            Serial.print(F(", "));
            Serial.print(accel_t_gyro.value.x_gyro/131.0, DEC);
            Serial.print(F(", "));
            Serial.print(accel_t_gyro.value.y_gyro, DEC);
            Serial.print(F(", "));
            Serial.print(accel_t_gyro.value.y_gyro/131.0, DEC);
            Serial.print(F(", "));
            Serial.print(accel_t_gyro.value.z_gyro, DEC);
            Serial.println(F(", "));
            Serial.println("$");
            Serial.println(accel_t_gyro.value.z_gyro/131.0, DEC);
            Serial.println(millis());
            Serial.print(F(", "));
            Serial.println(F(""));
        }

        delay(5);
    }
*/
}
#endif
