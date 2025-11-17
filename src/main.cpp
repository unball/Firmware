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
    // PIDController::setGains(2.0f, 0.1f, 0.001f);
    PIDController::setGains(1.02f, 0.1f, 0.00f);
    // PIDController::setGains(0.4683f, 0.0f, 0.0282);

    Serial.println("State-space control initialized.");
    Serial.print("Robot Number: ");Serial.println(RobotConfig::getRobotNumber());
    // Initialize motors, encoders, etc. here
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
    
    // if(Wifi::isCommunicationLost()){
    //     v_ref = 0;
    //     w_ref = 0;
    //     Motor::move(MOTOR_RIGHT, v_ref);
    //     Motor::move(MOTOR_LEFT, v_ref);
    // }

    // === Update state-space controller (runs every T internally) ===
    StateSpaceController::update(v_ref, w_ref);

    // PIDController::update(v_ref, w_ref);

    // === Send reference to adaptive controller ===
    // AdaptiveController::setReferences(
    //     PIDController::getOmegaLeft(),
    //     PIDController::getOmegaRight()
    // );

    // === Send reference to adaptive controller ===
    AdaptiveController::setReferences(
        StateSpaceController::getControlLeft(),
        StateSpaceController::getControlRight()
    );

    //    === Send reference to adaptive controller ===
    AdaptiveController::setReferences(
        ((v_ref - (L/2)*w_ref) / R),
        ((v_ref + (L/2)*w_ref) / R)
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

    // === Send feedback ===
    Wifi::sendFeedback(
        v, w,
        v_ref, w_ref,
        StateSpaceController::getControlLeft(),
        StateSpaceController::getControlRight(),
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

    // // === Print debug ===
    // if (RobotConfig::getRobotNumber() == 0) {
    //     Serial.print("v: "); Serial.print(v, 4);
    //     Serial.print(" | w: "); Serial.print(w, 4);
    //     Serial.print(" || u_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
    //     Serial.print(" | u_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
    //     Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
    //     Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
    // } else {
    //     Serial.print("ref_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
    //     Serial.print(" | ref_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
    //     Serial.print(" || LEFT: w_L "); Serial.print(AdaptiveController::getOmegaLeft(), 3);
    //     Serial.print(" | u_L: "); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
    //     Serial.print(" | theta1_L: "); Serial.print(AdaptiveController::getTheta1Left(), 3);
    //     Serial.print(" | theta2_L: "); Serial.print(AdaptiveController::getTheta2Left(), 3);
    //     Serial.print(" | e_L: "); Serial.print(AdaptiveController::getErrorLeft(), 2);
    //     Serial.print("  ||  RIGHT: w_R: "); Serial.print(AdaptiveController::getOmegaRight(), 3);
    //     Serial.print(" | u_R: "); Serial.print(AdaptiveController::getControlSignalRight(), 2);
    //     Serial.print(" | theta1_R: "); Serial.print(AdaptiveController::getTheta1Right(), 3);
    //     Serial.print(" | theta2_R: "); Serial.print(AdaptiveController::getTheta2Right(), 3);
    //     Serial.print(" | e_R: "); Serial.println(AdaptiveController::getErrorRight(), 2);
    // }
}
