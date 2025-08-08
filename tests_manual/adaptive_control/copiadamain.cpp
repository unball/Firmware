#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "robot_config.hpp"
#include "imu.hpp"
#include "adaptive_controller.h"
#include "wifi.hpp"

// === System parameters ===
// const float R = 0.021f;     // Wheel radius [m]
// const float L = 0.075f;     // Wheelbase [m]
const float T = 0.02f;      // Sampling time [s]
const float u_min = -69.0f; // Control saturation
const float u_max =  69.0f;

// === Gains from MATLAB (example with poles = [0.9, 0.9]) ===
const float K[2][2] = {
    {24.9952, -0.9373},
    {24.9952,  0.9373}
};

const float N[2][2] = {
    {72.1380f, -2.7052},
    {72.1380f,  2.7052}
};

// === State and reference ===
float v = 0.0f, w = 0.0f;                  // Measured linear and angular velocity
float v_ref = 0.105f, w_ref = 0.0f;          // Reference signals
float u_L = 0.0f, u_R = 0.0f;              // Control outputs

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    IMU::setup();
    RobotConfig::setup();
    Wifi::setup(RobotConfig::getRobotNumber());
    Serial.println("State-space control initialized.");
    Serial.print("Robot Number: ");Serial.println(RobotConfig::getRobotNumber());
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
        //    5.0,5.0
         u_L,
         u_R
        );

        AdaptiveController::update();

        Wifi::sendFeedback(
            v, w,
            u_L, u_R,
            omega_L, omega_R,
            AdaptiveController::getOmegaLeft(),
            AdaptiveController::getOmegaRight(),
            AdaptiveController::getTheta1Left(),
            AdaptiveController::getTheta2Left(),
            AdaptiveController::getTheta1Right(),
            AdaptiveController::getTheta2Right(),
            AdaptiveController::getErrorLeft(),
            AdaptiveController::getErrorRight()
        );



        // === Print debug ===
        if (RobotConfig::getRobotNumber() == 0) {
            Serial.print("v: "); Serial.print(v, 4);
            Serial.print(" | w: "); Serial.print(w, 4);
            Serial.print(" || u_L: "); Serial.print(u_L, 2);
            Serial.print(" | u_R: "); Serial.print(u_R, 2);
            Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
            Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
        } else {
            Serial.print("ref_L: "); Serial.print(u_L, 2);
            Serial.print(" | ref_R: "); Serial.print(u_R, 2);
            Serial.print(" || LEFT: w_L "); Serial.print(AdaptiveController::getOmegaLeft(), 3);
            Serial.print(" | u_L: "); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
            Serial.print(" | theta1_L: "); Serial.print(AdaptiveController::getTheta1Left(), 3);
            Serial.print(" | theta2_L: "); Serial.print(AdaptiveController::getTheta2Left(), 3);
            Serial.print(" | e_L: "); Serial.print(AdaptiveController::getErrorLeft(), 2);
            Serial.print("  ||  RIGHT: w_R: "); Serial.print(AdaptiveController::getOmegaRight(), 3);
            Serial.print(" | u_R: "); Serial.print(AdaptiveController::getControlSignalRight(), 2);
            Serial.print(" | theta1_R: "); Serial.print(AdaptiveController::getTheta1Right(), 3);
            Serial.print(" | theta2_R: "); Serial.print(AdaptiveController::getTheta2Right(), 3);
            Serial.print(" | e_R: "); Serial.println(AdaptiveController::getErrorRight(), 2);

        }
    }
}
