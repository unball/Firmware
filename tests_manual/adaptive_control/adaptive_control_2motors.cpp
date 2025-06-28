// File: src/main.cpp
#include <Arduino.h>
#include "encoder.h"
#include "motor.h"
#include "config.h"
#include "control.h"

// === Parameters ===
const float T = 0.02;
const float tau_m = 0.01;
const float gamma_adapt = 0.05;

// Reference model coefficients
const float am = exp(-T / tau_m);
const float bm = 1.0f - am;

// Reference limits
const float theta1_max = 25.0f;
const float theta1_min = -25.0f;
const float theta2_max = 3.5f;
const float theta2_min = -3.5f;

// Deadzone
const float motor_deadzone_c = 20.0f;
const float deadzone_threshold = 0.15f;

// Adaptive parameters
float theta1_L = (pwm_max * R) / 1.0f;
float theta2_L = 0.0f;
float theta1_R = (pwm_max * R) / 1.0f;
float theta2_R = 0.0f;

// System variables
float omega_L = 0.0f, omega_R = 0.0f;
float omega_m_L = 0.0f, omega_m_R = 0.0f;
float u_L = 0.0f, u_R = 0.0f;
float r = 0.0f;

// Timer
unsigned long last_time = 0;

// === Helpers ===
float applyDeadzone(float u_in, float dz_pos, float dz_neg) {
    if (u_in > 0.0f)
        return (u_in > dz_pos) ? u_in : dz_pos;
    else if (u_in < 0.0f)
        return (u_in < -dz_neg) ? u_in : -dz_neg;
    return 0.0f;
}

float applyDeadzoneError(float e, float delta) {
    if (e > delta) return e - delta;
    else if (e < -delta) return e + delta;
    else return 0.0f;
}

void update_adaptive_control() {
    Encoder::vel vel = Encoder::getMotorSpeeds();
    omega_L = vel.motorLeft;
    omega_R = vel.motorRight;

    // Referência: onda quadrada simétrica
    float t = millis() / 1000.0f;
    float period = 4.0f;
    float amplitude = 5.0f;
    float phase = fmod(t, period);
    r = (phase < period / 2.0f) ? amplitude : -amplitude;

    // === Roda Direita ===
    omega_m_R = am * omega_m_R + bm * r;
    float e_R = omega_R - omega_m_R;
    float e_eff_R = applyDeadzoneError(e_R, deadzone_threshold);
    float u_R_unsat = theta1_R * r - theta2_R * omega_R;
    u_R = constrain(u_R_unsat, (r >= 0 ? 0.0f : -100.0f), (r >= 0 ? 100.0f : 0.0f));

    float delta_theta1_R = -T * gamma_adapt * r * e_eff_R;
    float delta_theta2_R =  T * gamma_adapt * omega_R * e_eff_R;

    if ((theta1_R >= theta1_max && delta_theta1_R > 0) || (theta1_R <= theta1_min && delta_theta1_R < 0)) delta_theta1_R = 0;
    if ((theta2_R >= theta2_max && delta_theta2_R > 0) || (theta2_R <= theta2_min && delta_theta2_R < 0)) delta_theta2_R = 0;

    theta1_R += delta_theta1_R;
    theta2_R += delta_theta2_R;
    u_R = theta1_R * r - theta2_R * omega_R;
    float u_R_adj = applyDeadzone(u_R, motor_deadzone_c, motor_deadzone_c);
    Motor::move(MOTOR_RIGHT, u_R_adj);

    // === Roda Esquerda ===
    omega_m_L = am * omega_m_L + bm * r;
    float e_L = omega_L - omega_m_L;
    float e_eff_L = applyDeadzoneError(e_L, deadzone_threshold);
    float u_L_unsat = theta1_L * r - theta2_L * omega_L;
    u_L = constrain(u_L_unsat, (r >= 0 ? 0.0f : -100.0f), (r >= 0 ? 100.0f : 0.0f));

    float delta_theta1_L = -T * gamma_adapt * r * e_eff_L;
    float delta_theta2_L =  T * gamma_adapt * omega_L * e_eff_L;

    if ((theta1_L >= theta1_max && delta_theta1_L > 0) || (theta1_L <= theta1_min && delta_theta1_L < 0)) delta_theta1_L = 0;
    if ((theta2_L >= theta2_max && delta_theta2_L > 0) || (theta2_L <= theta2_min && delta_theta2_L < 0)) delta_theta2_L = 0;

    theta1_L += delta_theta1_L;
    theta2_L += delta_theta2_L;
    u_L = theta1_L * r - theta2_L * omega_L;
    float u_L_adj = applyDeadzone(u_L, motor_deadzone_c, motor_deadzone_c);
    Motor::move(MOTOR_LEFT, u_L_adj);

    // === Logging ===
    Serial.print("t:"); Serial.print(t);
    Serial.print(", r:"); Serial.print(r);

    Serial.print(", omega_R:"); Serial.print(omega_R);
    Serial.print(", omega_m_R:"); Serial.print(omega_m_R);
    Serial.print(", u_R:"); Serial.print(u_R);
    Serial.print(", theta1_R:"); Serial.print(theta1_R);
    Serial.print(", theta2_R:"); Serial.print(theta2_R);
    Serial.print(", e_R:"); Serial.print(e_R);

    Serial.print(", omega_L:"); Serial.print(omega_L);
    Serial.print(", omega_m_L:"); Serial.print(omega_m_L);
    Serial.print(", u_L:"); Serial.print(u_L);
    Serial.print(", theta1_L:"); Serial.print(theta1_L);
    Serial.print(", theta2_L:"); Serial.print(theta2_L);
    Serial.print(", e_L:"); Serial.println(e_L);
}

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
}

void loop() {
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(T * 1000)) {
        last_time = now;
        update_adaptive_control();
    }
}
