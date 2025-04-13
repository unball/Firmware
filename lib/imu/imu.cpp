#include "imu.hpp"

namespace IMU {

    Adafruit_LSM6DS3TRC imu;

    void setup() {
    	Wire.begin(SDA_PIN, SCL_PIN);

        imu.begin_I2C(IMU_I2C_ADDR);

        // Configure accelerometer and gyro ranges
        imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
        imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

        // Configure data rates (also affects internal filtering)
        imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
        imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    }

    float get_w() {
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        return gyro.gyro.z;
    }

}
