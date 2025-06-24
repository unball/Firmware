// File: src/main.cpp
#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "control.hpp"

// === Parameters ===
const float T = 0.02;               // Sampling time [s]
const float tau_m = 0.01;           // Reference model time constant [s]
const float gamma_adapt = 0.090;     // Adaptation gain
//  0.00025 (Geovany)

// Reference model coefficients
const float am = exp(-T / tau_m);
const float bm = 1.0f - am;

// // Adaptive parameters
// float theta1 = 1.5f;
// float theta2 = 1.2f;

// System variables
float ommega = 0.0f;
float ommega_m = 0.0f;
float u = 0.0f;
float r = 0.0f;

// Timer control
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
}

float applyDeadzone(float u_in, float dz_positive = 15.0f, float dz_negative = 15.0f) {
    if (u_in > 0.0f)
        return (u_in > dz_positive) ? u_in : dz_positive;
    else if (u_in < 0.0f)
        return (u_in < -dz_negative) ? u_in : -dz_negative;
    return 0.0f;
}

void update_adaptive_control() {
    // Adaptive parameters
    static float theta1 = (pwm_max*R)/v_max;
    static float theta2 = 0.0;

    // Reference limits
    const float theta1_max = 50.0f;
    const float theta2_min = -50.0f;
    const float theta2_max = 4.0f; // limite seguro extraído dos logs

    // Read encoder
    Encoder::vel vel = Encoder::getMotorSpeeds();
    ommega = vel.motorRight;

    // Apply low-pass filter to encoder reading
    static float ommega_filtered = 0.0f;
    const float alpha = 0.2f;
    ommega_filtered = alpha * ommega + (1.0f - alpha) * ommega_filtered;
    ommega = ommega_filtered;

    // Generate square wave reference input
    float t = millis() / 1000.0f;  // Tempo em segundos
    float period = 4.0f;           // Período total (sobe e desce)
    float amplitude = 5.0f;

    float phase = fmod(t, period);  // Tempo dentro do período
    r = (phase < period / 2.0f) ? amplitude : -amplitude;

    // Reference model update
    ommega_m = am * ommega_m + bm * r;

    // Compute error
    float e = ommega - ommega_m;

    // Compute control signal (unsaturated)
    float u_unsat = theta1 * r - theta2 * ommega;

    // Saturate u
    if (r >= 0.0f)
        u = constrain(u_unsat, 0.0f, 100.0f);
    else
        u = constrain(u_unsat, -100.0f, 0.0f);

    // Adaptation terms
    float delta_theta1 = -T * gamma_adapt * r * e;
    float delta_theta2 =  T * gamma_adapt * ommega * e;

    // Update theta1 with simple constraint
    theta1 = constrain(theta1 + delta_theta1, -theta1_max, theta1_max);

    // === Projection for theta2 ===
    if ((theta2 >= theta2_max && delta_theta2 > 0.0f) ||
        (theta2 <= theta2_min && delta_theta2 < 0.0f)) {
        delta_theta2 = 0.0f; // block when pushing beyond bounds
    }
    theta2 += delta_theta2;

    // Recompute u with updated thetas
    u = theta1 * r - theta2 * ommega;

    // Apply deadzone
    float u_adj = applyDeadzone(u);
    Motor::move(MOTOR_RIGHT, u_adj);

    // Log data
    Serial.print("t:"); Serial.print(t);
    Serial.print(", r:"); Serial.print(r);
    Serial.print(", ommega:"); Serial.print(ommega);
    Serial.print(", ommega_m:"); Serial.print(ommega_m);
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