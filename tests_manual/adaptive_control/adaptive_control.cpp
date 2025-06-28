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

// Reference model coefficients
const float am = exp(-T / tau_m);
const float bm = 1.0f - am;

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

float applyDeadzone(float u_in, float dz_positive = 20.0f, float dz_negative = 20.0f) {
    if (u_in > 0.0f)
        return (u_in > dz_positive) ? u_in : dz_positive;
    else if (u_in < 0.0f)
        return (u_in < -dz_negative) ? u_in : -dz_negative;
    return 0.0f;
}

void update_adaptive_control() {
    // Adaptive parameters
    static float theta1 = (pwm_max * R) / 1.0f;
    static float theta2 = 0.0;

    // Reference limits
    const float theta1_max = 25.0f;
    const float theta1_min = -25.0f;
    const float theta2_max = 3.5f;
    const float theta2_min = -3.5f;

    // Read encoder
    Encoder::vel vel = Encoder::getMotorSpeeds();
    ommega = vel.motorRight;

    // Generate square wave reference
    float t = millis() / 1000.0f;
    float period = 4.0f;
    float amplitude = 5.0f;
    float phase = fmod(t, period);
    r = (phase < period / 2.0f) ? amplitude : -amplitude;

    // Reference model
    ommega_m = am * ommega_m + bm * r;

    // Compute tracking error
    float e = ommega - ommega_m;

    // Apply deadzone method: only adapt if |e| > threshold
    const float deadzone_threshold = 0.15f;
    if (fabs(e) > deadzone_threshold) {
        float delta_theta1 = -T * gamma_adapt * r * e;
        float delta_theta2 =  T * gamma_adapt * ommega * e;

        // Projection for theta1
        if (!((theta1 >= theta1_max && delta_theta1 > 0.0f) ||
              (theta1 <= theta1_min && delta_theta1 < 0.0f))) {
            theta1 += delta_theta1;
        }

        // Projection for theta2
        if (!((theta2 >= theta2_max && delta_theta2 > 0.0f) ||
              (theta2 <= theta2_min && delta_theta2 < 0.0f))) {
            theta2 += delta_theta2;
        }
    }

    // Compute control signal
    float u_unsat = theta1 * r - theta2 * ommega;

    // Saturate control
    u = constrain(u_unsat, (r >= 0.0f ? 0.0f : -100.0f), (r >= 0.0f ? 100.0f : 0.0f));

    // Apply motor deadzone
    float u_adj = applyDeadzone(u);
    Motor::move(MOTOR_RIGHT, u_adj);

    // Logging
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
