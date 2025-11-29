#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "robot_config.hpp"
#include "imu.hpp"
#include "pid_controller.hpp"
#include "adaptive_controller.h"
#include "wifi.hpp"
#include "control.hpp"
#include "state_space_controller.hpp"

// === State and reference ===
float v_ref = 0.0f, w_ref = 0.0f;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    IMU::setup();
    RobotConfig::setup();
    Wifi::setup(RobotConfig::getRobotNumber());
}

void loop() {
    // Se for o robô 1, executa Twiddle ao invés da rotina normal
    if (RobotConfig::getRobotNumber() == 1) {
        
        Serial.println("Starting Twiddle algorithm...");
        delay(1000);
        PIDController::twiddle();
        Motor::stop();
        while (1);
        return;
    }

    static int16_t v_int;
    static int16_t w_int;

    // === Receive references from Wi-Fi ===
    Wifi::receiveData(&v_int, &w_int);

    v_ref = ((float)v_int) * 2.0f / 32767;
    w_ref = ((float)w_int) * 64.0f / 32767;

    // StateSpaceController::update(v_ref, w_ref);
    // PIDController::update(v_ref, w_ref);

    // === Send reference to adaptive controller ===
    AdaptiveController::setReferences(
        ((v_ref - (L/2)*w_ref) / R),
        ((v_ref + (L/2)*w_ref) / R)
        // PIDController::getOmegaLeft(),
        // PIDController::getOmegaRight()
        // StateSpaceController::getControlLeft(),
        // StateSpaceController::getControlRight()
    );

    AdaptiveController::update();

    // === Collect measurements ===
    Encoder::vel vel = Encoder::getMotorSpeeds();
    float omega_L = vel.motorLeft;
    float omega_R = vel.motorRight;
    float v = (R / 2.0f) * (omega_R + omega_L);
    // float w = IMU::get_w();
    float w_encoders = Encoder::getAngularVelocity(R, L);
    float w = IMU::get_w_filtered(w_encoders , 0.98f);

    // === Send feedback (throttled to 50 Hz) ===
    static unsigned long lastFeedback = 0;
    if (millis() - lastFeedback >= 10) {
        Wifi::sendFeedback(
            v, w,
            v_ref, w_ref,
            ((v_ref - (L/2)*w_ref) / R), ((v_ref + (L/2)*w_ref) / R),
            // PIDController::getOmegaLeft(),                                                              // omega_ref_L // PIDController::getOmegaRight(),  // omega_ref_R
            // StateSpaceController::getControlLeft(), StateSpaceController::getControlRight(),                                                     //  // omega_ref_R
            omega_L, omega_R,                                                                           // omega L measured by main, omega R measured by main
            AdaptiveController::getControlSignalLeft(), AdaptiveController::getControlSignalRight(),    // u_R = theta1_R * r_R - theta2_R * omega_R
            AdaptiveController::getTheta1Left(), AdaptiveController::getTheta2Left(),
            AdaptiveController::getTheta1Right(), AdaptiveController::getTheta2Right(), 
            AdaptiveController::getErrorLeft(), AdaptiveController::getErrorRight()
        );
        lastFeedback = millis();
    }

    // === Print debug ===
    if (!RobotConfig::getRobotNumber() == 0) {
        Serial.print(F("v: ")); Serial.print(v, 4);
        Serial.print(F(" | w: ")); Serial.print(w, 4);
        Serial.print(F(" || u_L: ")); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
        Serial.print(F(" | u_R: ")); Serial.print(AdaptiveController::getControlSignalRight(), 2);
        Serial.print(F(" || omega_L: ")); Serial.print(omega_L, 2);
        Serial.print(F(" | omega_R: ")); Serial.println(omega_R, 2);
    } else {
        Serial.print(F("ref_L: ")); Serial.print(((v_ref - (L/2)*w_ref) / R), 2);
        Serial.print(F(" | ref_R: ")); Serial.print(((v_ref + (L/2)*w_ref) / R), 2);
        Serial.print(F(" || LEFT: w_L ")); Serial.print(AdaptiveController::getOmegaLeft(), 3);
        Serial.print(F(" | u_L: ")); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
        Serial.print(F(" | theta1_L: ")); Serial.print(AdaptiveController::getTheta1Left(), 3);
        Serial.print(F(" | theta2_L: ")); Serial.print(AdaptiveController::getTheta2Left(), 3);
        Serial.print(F(" | e_L: ")); Serial.print(AdaptiveController::getErrorLeft(), 2);
        Serial.print(F("  ||  RIGHT: w_R: ")); Serial.print(AdaptiveController::getOmegaRight(), 3);
        Serial.print(F(" | u_R: ")); Serial.print(AdaptiveController::getControlSignalRight(), 2);
        Serial.print(F(" | theta1_R: ")); Serial.print(AdaptiveController::getTheta1Right(), 3);
        Serial.print(F(" | theta2_R: ")); Serial.print(AdaptiveController::getTheta2Right(), 3);
        Serial.print(F(" | e_R: ")); Serial.println(AdaptiveController::getErrorRight(), 2);
    }
}
