#include "pid_controller.hpp"
#include "imu.hpp"
#include <Arduino.h>

// === Robot parameters ===
static constexpr float R = 0.0215f;   // Wheel radius [m]
static constexpr float L = 0.0825f;   // Wheelbase [m]

// === PID gains ===
static float Kp = 2.7f;
static float Ki = 0.1f;
static float Kd = -0.08f;

// === Internal variables ===
static float integral = 0.0f;
static float lastError = 0.0f;
static unsigned long lastTime = 0;

// === Wheel references ===
static float omega_L_ref = 0.0f;
static float omega_R_ref = 0.0f;

namespace PIDController {

    void reset() {
        integral = 0.0f;
        lastError = 0.0f;
        omega_L_ref = 0.0f;
        omega_R_ref = 0.0f;
        lastTime = millis();
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void update(float v_ref, float w_ref) {
        // Ensure fixed sampling
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.0f) return;
        lastTime = now;

        // === Measure angular velocity from IMU ===
        float w_meas = IMU::get_w();

        // === Compute error ===
        float error = w_ref - w_meas;

        // === PID terms ===
        integral += error * dt;
        float derivative = (error - lastError) / dt;

        float w_control = Kp * error + Ki * integral + Kd * derivative;
        lastError = error;

        // === Convert (v_ref, w_control) to wheel references ===
        omega_R_ref = (v_ref / R) + (L / (2.0f * R)) * w_control;
        omega_L_ref = (v_ref / R) - (L / (2.0f * R)) * w_control;
    }

    float getOmegaLeft()  { return omega_L_ref; }
    float getOmegaRight() { return omega_R_ref; }

}
