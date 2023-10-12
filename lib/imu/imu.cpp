#include "imu.hpp"

namespace IMU{

    Adafruit_MPU6050 mpu;

    void setup(){
        if (!mpu.begin()) {
            #if WEMOS_DEBUG
            Serial.println("Failed to find MPU6050 chip");
            while (1) {
                delay(10);
            }
            #endif
            bool control = false;
        }
        #if WEMOS_DEBUG
        Serial.println("MPU6050 Found!");
        #endif
        
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    float get_w(){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float theta = g.gyro.z; 
        return theta;
    }
}