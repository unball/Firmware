#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "imu.hpp"
#include "adaptive_controller.h"

// === System parameters ===
// const float R = 0.021f;     // Wheel radius [m]
// const float L = 0.075f;     // Wheelbase [m]
const float T = 0.02f;      // Sampling time [s]
const float u_min = -69.0f; // Control saturation
const float u_max =  69.0f;

// === Gains from MATLAB (example with poles = [0.9, 0.9]) ===
const float K[2][2] = {
    {238.0952f, -8.9286f},
    {238.0952f,  8.9286f}
};

const float N[2][2] = {
    {238.0952f, -8.9286f},
    {238.0952f,  8.9286f}
};

// === State and reference ===
float v = 0.0f, w = 0.0f;                  // Measured linear and angular velocity
float v_ref = 0.1f, w_ref = 0.0f;          // Reference signals
float u_L = 0.0f, u_R = 0.0f;              // Control outputs

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    IMU::setup();
    AdaptiveController::setup();
    Serial.println("State-space control initialized.");
    // Initialize motors, encoders, etc. here
}

void loop() {
    static unsigned long lastTime = 0;
    if (millis() - lastTime >= T * 1000) {
        lastTime = millis();

        // === Read measured velocities (from encoder/IMU) ===
        Encoder::vel vel = Encoder::getMotorSpeeds();
        float omega_L = vel.motorLeft;
        float omega_R = vel.motorRight;

        float v = (R / 2.0f) * (omega_R + omega_L);
        float w = 0.0f;  // velocidade angular do robô
        float x[2] = {v, w};
        float r[2] = {v_ref, w_ref};

        // === Compute control: u = -Kx + Nr ===
        float u_unsat[2];
        for (int i = 0; i < 2; i++) {
            u_unsat[i] = 0.0f;
            for (int j = 0; j < 2; j++) {
                u_unsat[i] += -K[i][j] * x[j] + N[i][j] * r[j];
            }
        }

        // === Apply saturation ===
        u_L = constrain(u_unsat[0], u_min, u_max);
        u_R = constrain(u_unsat[1], u_min, u_max);

        // === Send control to motors ===
        // setMotorSpeed(leftMotor, u_L);
        // setMotorSpeed(rightMotor, u_R);
        AdaptiveController::setReferences(
          10,10
          // u_L,
          // u_R
        );

        AdaptiveController::bypassControl();

        // === Print debug ===
        Serial.print("v: "); Serial.print(v, 4);
        Serial.print(" | w: "); Serial.print(w, 4);
        Serial.print(" || u_L: "); Serial.print(u_L, 2);
        Serial.print(" | u_R: "); Serial.print(u_R, 2);
        Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
        Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
    }
}
