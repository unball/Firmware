#include "imu.hpp"

namespace IMU {

    Adafruit_LSM6DS3TRC imu;
    static float w_filtered_prev = 0.0f;  // Previous filtered value

    void setup() {
    	Wire.begin(SDA_PIN, SCL_PIN);

        imu.begin_I2C(IMU_I2C_ADDR);

        // Configure accelerometer and gyro ranges
        imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
        imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

        // Configure data rates (also affects internal filtering)
        imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
        imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
        
        w_filtered_prev = 0.0f;
    }

    float get_w() {
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        return gyro.gyro.z;
    }

    /**
     * @brief Complementary filter for angular velocity
     * Combines high-frequency gyro data with low-frequency encoder-based estimation
     * 
     * @param w_encoders Angular velocity calculated from wheel encoders: (v_right - v_left) / L
     * @param alpha Weight for gyro (high-pass). Typical: 0.95-0.98. Higher = trust gyro more
     * @return Filtered angular velocity in rad/s
     * 
     * Formula: w_filtered = alpha * w_gyro + (1 - alpha) * w_encoders
     * 
     * - w_gyro: Good for fast changes, but drifts over time
     * - w_encoders: More accurate long-term, but slower response
     */
    float get_w_filtered(float w_encoders, float alpha) {
        float w_gyro = get_w();
        
        // Complementary filter
        w_filtered_prev = alpha * w_gyro + (1.0f - alpha) * w_encoders;
        
        return w_filtered_prev;
    }

}
