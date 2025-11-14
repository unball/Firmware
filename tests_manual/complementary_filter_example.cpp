/**
 * @file complementary_filter_example.cpp
 * @brief Example of using complementary filter for angular velocity
 * 
 * This example demonstrates how to combine IMU gyroscope data with
 * encoder-based angular velocity estimation using a complementary filter.
 */

#include <Arduino.h>
#include "encoder.hpp"
#include "imu.hpp"
#include "config.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize sensors
    Encoder::setup();
    IMU::setup();
    
    Serial.println("=== Complementary Filter Example ===");
    Serial.println("Columns: time(ms), w_gyro, w_encoders, w_filtered");
}

void loop() {
    // Get raw angular velocity from gyroscope
    float w_gyro = IMU::get_w();
    
    // Calculate angular velocity from encoders
    float w_encoders = Encoder::getAngularVelocity(R, L);
    
    // Apply complementary filter
    // alpha = 0.98 means: 98% gyro (fast response) + 2% encoders (drift correction)
    float alpha = 0.98;
    float w_filtered = IMU::get_w_filtered(w_encoders, alpha);
    
    // Print results
    Serial.printf("%lu, %.4f, %.4f, %.4f\n", 
                  millis(), 
                  w_gyro, 
                  w_encoders, 
                  w_filtered);
    
    delay(10);  // 100 Hz sampling rate
}


/* ==================== USAGE NOTES ====================
 * 
 * COMPLEMENTARY FILTER EXPLANATION:
 * ---------------------------------
 * w_filtered = alpha * w_gyro + (1 - alpha) * w_encoders
 * 
 * ALPHA VALUE TUNING:
 * - alpha = 0.95 to 0.98 (typical)
 * - Higher alpha (closer to 1.0):
 *   + Faster response to changes
 *   + More trust in gyro
 *   - More susceptible to gyro drift
 * 
 * - Lower alpha (closer to 0.5):
 *   + Better long-term accuracy
 *   + More trust in encoders
 *   - Slower response
 * 
 * WHEN TO USE EACH SENSOR ALONE:
 * ------------------------------
 * Use w_gyro (IMU::get_w()) when:
 * - Very fast dynamics (quick turns)
 * - Short duration maneuvers
 * - Need instant response
 * 
 * Use w_encoders (Encoder::getAngularVelocity()) when:
 * - Slow, steady motion
 * - Long duration operation
 * - Accuracy is more important than speed
 * 
 * Use w_filtered (IMU::get_w_filtered()) when:
 * - General robot control
 * - Balance between speed and accuracy needed
 * - Want drift-free gyro measurements
 * 
 * INTEGRATION IN CONTROL LOOPS:
 * -----------------------------
 * Example 1: Replace w in PID controller
 * 
 *   float w_encoders = Encoder::getAngularVelocity();
 *   float w = IMU::get_w_filtered(w_encoders, 0.98);
 *   
 *   // Use w in your control algorithm
 *   PIDController::update(v_ref, w);
 * 
 * Example 2: Use in state feedback
 * 
 *   float w_encoders = Encoder::getAngularVelocity();
 *   float w_filtered = IMU::get_w_filtered(w_encoders);
 *   
 *   // State vector: [v, w, ...]
 *   state[1] = w_filtered;
 * 
 * Example 3: Adaptive alpha based on motion
 * 
 *   float w_encoders = Encoder::getAngularVelocity();
 *   float alpha = (abs(w_encoders) > 2.0) ? 0.98 : 0.90;
 *   float w = IMU::get_w_filtered(w_encoders, alpha);
 * 
 * TROUBLESHOOTING:
 * ---------------
 * - If w_filtered drifts: Decrease alpha (try 0.95 or 0.90)
 * - If w_filtered is too slow: Increase alpha (try 0.99)
 * - If w_filtered oscillates: Check encoder noise, may need to filter w_encoders first
 * - If encoders give 0 at low speeds: This is normal due to watchdog timer
 * 
 * PHYSICAL PARAMETERS:
 * -------------------
 * From config.h:
 * - R (wheel radius) = 0.0215 m
 * - L (wheelbase) = 0.0825 m
 * 
 * These are used in: w_encoders = (v_right - v_left) / L
 * where v_right = omega_right * R
 * 
 */