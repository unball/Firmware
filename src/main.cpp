// File: src/main.cpp
#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"

// === Parameters ===
const float T = 0.02;               // Sampling time [s]
const float tau_m = 0.05;           // Reference model time constant [s]
const float gamma_adapt = 0.001;     // Adaptation gain

// Reference model coefficients
const float am = exp(-T / tau_m);
const float bm = 1.0f - am;

// Adaptive parameters
float theta1 = 0.5f;
float theta2 = 0.2f;

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

void update_adaptive_control() {
    // Read encoder
    Encoder::vel vel = Encoder::getMotorSpeeds();
    y = vel.motorLeft;

    // Apply low-pass filter to encoder reading
    static float y_filtered = 0.0f;
    const float alpha = 0.2f;
    y_filtered = alpha * y + (1.0f - alpha) * y_filtered;
    y = y_filtered;

    // Generate reference input
    float t = millis() / 1000.0f;
    r = (t > 1.0f) ? 8.0f : 0.0f;

    // Reference model update
    ym = am * ym + bm * r;

    // Compute error
    float e = y - ym;

    // Compute control signal
    u = theta1 * r - theta2 * y;

    // Apply saturation
    float u_unsat = u;
    u = constrain(u, -100.0f, 100.0f);

    // Anti-windup: only adapt if control not saturated
    //if (fabs(u_unsat - u) < 1e-3) {
        theta1 = constrain(theta1 - gamma_adapt * r * e, -50.0f, 50.0f);
        theta2 = constrain(theta2 + gamma_adapt * y * e, -50.0f, 50.0f);
    //}

    // Apply control
    Motor::move(MOTOR_RIGHT, u);
    Motor::move(MOTOR_LEFT, 100);

    // Log data
    Serial.print("t:"); Serial.print(t);
    Serial.print(", r:"); Serial.print(r);
    Serial.print(", y:"); Serial.print(y);
    Serial.print(", ym:"); Serial.print(ym);
    Serial.print(", u:"); Serial.print(u);
    Serial.print(", theta1:"); Serial.print(theta1);
    Serial.print(", theta2:"); Serial.println(theta2);
}

void loop() {
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(T * 1000)) {
        last_time = now;
        update_adaptive_control();
    }
}

