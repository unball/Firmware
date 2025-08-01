#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "control.hpp"
#include "driver/mcpwm.h"

// === Parameters ===
const double T = 0.01;               // Sampling time [s]
const double tau_m = 0.01;           // Reference model time constant [s]
const double gamma_adapt = 0.05;     // Adaptation gain

// Reference model coefficients
const double am = exp(-T / tau_m);
const double bm = 1.0f - am;

// System variables
double omega = 0.0f;
double omega_m = 0.0f;
double u = 0.0f;
double r = 0.0f;
const double deadzone = 73;

// Reference limits
const double theta1_limit = 250;
const double theta2_limit = 250;
// E-modification parameter
const double sigma = gamma_adapt/2;

// Timer control
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
}

double applyDeadzone(double u_in, double dz_positive = 17.0f, double dz_negative = 17.0f) {
    if (u_in > 0.0f)
        return (u_in > dz_positive) ? u_in : dz_positive;
    else if (u_in < 0.0f)
        return (u_in < -dz_negative) ? u_in : -dz_negative;
    return 0.0f;
}

void update_adaptive_control() {
    // Adaptive parameters
    // static double theta1 = 2*(((pwm_max + 0) * R) / v_max);     // estabilizam
    // static double theta2 = 0.3*(((pwm_max + 0) * R) / v_max);
    static double theta1 = ((pwm_max + deadzone) * R) / v_max;
    static double theta2 = theta1 - 23; // Experimentalmente verificou-se que quando theta1 - theta2 estabilizam, a diferença entre eles é de 23.

    // Read encoder
    Encoder::vel vel = Encoder::getMotorSpeeds();
    omega = vel.motorLeft;

    // Generate square wave reference
    double t = millis() / 1000.0f;
    double period = 4.0f;
    double amplitude = 10.0f;
    double phase = fmod(t, period);
    // r = amplitude;  //step
    r = (phase < period / 2.0f) ? amplitude : -amplitude;
    // r = (amplitude / period) * phase;  // Sawtooth ramp from 0 to amplitude

    // Reference model
    omega_m = am * omega_m + bm * r;

    // Compute tracking error
    double e = omega - omega_m;

    // Apply deadzone method: only adapt if |e| > threshold
    const double deadzone_threshold = 0.01f;
    if (fabs(e) > deadzone_threshold) {
        double delta_theta1 = -T * (gamma_adapt * r * e + sigma * fabs(e) * theta1);
        double delta_theta2 = T * (gamma_adapt * omega * e - sigma * fabs(e) * theta2);

        // Projection for theta1
        if (!((theta1 >= theta1_limit && delta_theta1 > 0.0f) ||
              (theta1 <= -theta1_limit && delta_theta1 < 0.0f))) {
            theta1 += delta_theta1;
        }

        // Projection for theta2
        if (!((theta2 >= theta2_limit && delta_theta2 > 0.0f) ||
              (theta2 <= -theta2_limit && delta_theta2 < 0.0f))) {
            theta2 += delta_theta2;
        }
    }

    // Compute control signal
    double u_unsat = theta1 * r - theta2 * omega;

    // Saturate control
    u = constrain(u_unsat, (r >= 0.0f ? 0.0f : -410.0f), (r >= 0.0f ? 410.0f : 0.0f));
    // u = constrain(u_unsat, -100.0f, 100.0f);

    // Apply motor deadzone
    double u_adj = applyDeadzone(u, deadzone, -deadzone);
    Motor::move(MOTOR_LEFT, u_adj);

    // Logging
    Serial.print("t:"); Serial.print(t);
    Serial.print(", r:"); Serial.print(r);
    Serial.print(", omega:"); Serial.print(omega);
    Serial.print(", omega_m:"); Serial.print(omega_m);
    Serial.print(", u:"); Serial.print(u);
    Serial.print(", theta1:"); Serial.print(theta1, 3);
    Serial.print(", theta2:"); Serial.print(theta2, 3);
    Serial.print(", e:"); Serial.println(e, 3);
}

void loop() {
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(T * 1000)) {
        last_time = now;
        update_adaptive_control();
    }
}