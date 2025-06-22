// File: src/main.cpp
#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "control.hpp"

// === Parameters ===
const float T = 0.02;               // Sampling time [s]
const float tau_m = 0.01;           // Reference model time constant [s]
const float gamma_adapt = 0.05;     // Adaptation gain
// gamma = 0.05, 0.005 e 0.0005
//  0.00025 (Geovany)

// Reference model coefficients
const float am = exp(-T / tau_m);
const float bm = 1.0f - am;

// // Adaptive parameters
// float theta1 = 1.5f;
// float theta2 = 1.2f;

// System variables
float y = 0.0f;
float ym = 0.0f;
float u = 0.0f;
float r = 0.0f;

// Timer control
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
}

float applyDeadzone(float u, float dz_positive = 15.0f, float dz_negative = 15.0f) {
    if (u > 0.0f)
        return (u > dz_positive) ? u : dz_positive;
    else if (u < 0.0f)
        return (u < -dz_negative) ? u : -dz_negative;
    return 0.0f;
}

void update_adaptive_control() {
    // Adaptive parameters
    static float theta1 = (pwm_max*R)/v_max;
    static float theta2 = 0.2;//(pwm_max*(Encoder::getMotorSpeeds().motorRight))/v_max;

    // Read encoder
    Encoder::vel vel = Encoder::getMotorSpeeds();
    y = vel.motorRight;

    // Apply low-pass filter to encoder reading
    static float y_filtered = 0.0f;
    const float alpha = 0.2f;
    y_filtered = alpha * y + (1.0f - alpha) * y_filtered;
    y = y_filtered;

    // Generate square wave reference input
    float t = millis() / 1000.0f;  // Tempo em segundos
    float period = 4.0f;           // Período total (sobe e desce)
    float amplitude = 5.0f;

    float phase = fmod(t, period);  // Tempo dentro do período
    r = (phase < period / 2.0f) ? amplitude : -amplitude;

    // Reference model update
    ym = am * ym + bm * r;

    // Compute error
    float e = y - ym;

    // Compute control signal
    u = theta1 * r - theta2 * y;

    // Apply directional constraint to u
    if (r >= 0.0f)
        u = constrain(u, 0.0f, 100.0f);
    else
        u = constrain(u, -100.0f, 0.0f);

    float u_unsat = theta1 * r - theta2 * y;

    // Anti-windup: only adapt if control not saturated
    // if (fabs(u_unsat - u) < 1e-3) {
        theta1 = constrain(theta1 - T * gamma_adapt * r * e, -50.0f, 50.0f);
        theta2 = constrain(theta2 + T * gamma_adapt * y * e, -50.0f, 4.5f);
    // }

    // Apply control
    float u_adj = applyDeadzone(u);
    Motor::move(MOTOR_RIGHT, u_adj);

    // Log data
    Serial.print("t:"); Serial.print(t);
    Serial.print(", r:"); Serial.print(r);
    Serial.print(", y:"); Serial.print(y);
    Serial.print(", ym:"); Serial.print(ym);
    Serial.print(", u:"); Serial.print(u);
    Serial.print(", theta1:"); Serial.print(theta1);
    Serial.print(", theta2:"); Serial.print(theta2);
    Serial.print(", e:"); Serial.println(e);
}

void loop() {
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(T * 1000)) {
        last_time = now;
        update_adaptive_control();
    }
}