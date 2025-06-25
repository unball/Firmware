#include "state_space_controller.h"
#include "encoder.hpp"
#include "IMU.hpp"
#include "motor.hpp"
#include "control.hpp"

StateSpaceController::StateSpaceController(float K11, float K12, float K21, float K22, float R, float L)
  : wheel_radius(R), wheel_base(L), last_update_us(0)
{
    K[0][0] = K11;
    K[0][1] = K12;
    K[1][0] = K21;
    K[1][1] = K22;
}

void printAlignedFloat(float value, int width, int decimals) {
    char buffer[20];
    dtostrf(value, width, decimals, buffer); // width inclui sinal, espaço, etc.
    Serial.print(buffer);
}

void StateSpaceController::update(float v_ref, float w_ref) {
    unsigned long now = micros();
    if (now - last_update_us >= Ts_us) {
        last_update_us = now;
        computeControl(v_ref, w_ref);
    }
}

void StateSpaceController::computeControl(float v_ref, float w_ref) {
    // Get actual values
    Encoder::vel vel = Encoder::getMotorSpeeds();  // in rad/s
    float omega_L = vel.motorLeft;
    float omega_R = vel.motorRight;

    // Estimate linear and angular velocity
    float v = (wheel_radius / 2.0f) * (omega_R + omega_L);
    float w = IMU::get_w();

    // Calculate errors
    float e_v = v_ref - v;
    float e_w = w_ref - w;

    // Control law: u = K * e
    float v_d = K[0][0] * e_v + K[0][1] * e_w;
    float w_d = K[1][0] * e_v + K[1][1] * e_w;

    // Inverse kinematics: convert to wheel references
    omega_L_d = (1.0f / wheel_radius) * (v_d - (wheel_base / 2.0f) * w_d);
    omega_R_d = (1.0f / wheel_radius) * (v_d + (wheel_base / 2.0f) * w_d);

    // Debug output
Serial.print("[REF] v:"); printAlignedFloat(v_ref, 5, 2);
Serial.print(" | w:"); printAlignedFloat(w_ref, 6, 2);
Serial.print(" || [MEAS] v:"); printAlignedFloat(v, 6, 2);
Serial.print(" | w:"); printAlignedFloat(w, 6, 2);
Serial.print(" || [e_v, e_w]: "); printAlignedFloat(e_v, 6, 2); Serial.print(", "); printAlignedFloat(e_w, 6, 2);
Serial.print(" || [u] v_d:"); printAlignedFloat(v_d, 6, 2);
Serial.print(" | w_d:"); printAlignedFloat(w_d, 6, 2);
Serial.print(" || [ωL_r, ωR_r]:");
printAlignedFloat(omega_L_d, 5, 2); Serial.print(",");
printAlignedFloat(omega_R_d, 5, 2);
Serial.print(" || [ωL_meas, ωR_meas]:");
printAlignedFloat(omega_L, 6, 2); Serial.print(",");
printAlignedFloat(omega_R, 6, 2); Serial.println();
}

float StateSpaceController::getOmegaRefLeft() const {
    return omega_L_d;
}

float StateSpaceController::getOmegaRefRight() const {
    return omega_R_d;
}
