#include "pid_controller.hpp"
#include "wifi.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "..\..\include\config.h"
#include <Arduino.h>
#include "adaptive_controller.h"    


namespace PIDController {

    // === Twiddle parameters ===
    static float dk[3] = {0.5f, 0.05f, 0.05f};  // initial deltas
    static float ksi = 0.3f;                    // learning rate
    static float epsilon = 1e-3f;               // stop threshold
    static int maxIterations = 30;              // safety limit

    // === Internal variable for error tracking ===
    static float maxError = 0.0f;

    // === Robot parameters ===
    static constexpr float T = 0.02f;     // Sampling period [s]

    // === PID gains ===
    static float Kp = 1.5f;
    static float Ki = 0.05f;
    static float Kd = -0.05f;

    // === Internal variables ===
    static float integral = 0.0f;
    static float lastError = 0.0f;
    static unsigned long lastTime = 0;

    // === Wheel references ===
    static float omega_L_ref = 0.0f;
    static float omega_R_ref = 0.0f;

    void reset() {
        integral = 0.0f;
        lastError = 0.0f;
        omega_L_ref = 0.0f;
        omega_R_ref = 0.0f;
        lastTime = millis();
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void update(float v_ref, float w_ref) {
        unsigned long now = millis();

        // Check if it is time to run the control step
        if (now - lastTime < (unsigned long)(T * 1000.0f)) {
            return; // Skip if period not reached
        }
        lastTime = now;

        // if(Wifi::isCommunicationLost()){
        //     reset();
        //     Motor::stop();
        // }
        
        // if (v_ref == 0 && w_ref == 0) {
        //     Motor::stop();
        //     reset();
        // }

        // === Measure angular velocity from IMU ===
        float w_meas = IMU::get_w();

        // === Compute error ===
        float error = w_ref - w_meas;

        // === PID terms ===
        integral += error;
        integral = (abs(integral) < 64.0f) ?  integral : 0; // Anti-windup

        float derivative = (error - lastError);

        float w_control = Kp * error + Ki * integral + Kd * derivative;
        lastError = error;

        // === Convert (v_ref, w_control) to wheel references ===
        omega_R_ref = (v_ref + (L/2)*w_control) / R;
        omega_L_ref = (v_ref - (L/2)*w_control) / R;
    }

    void twiddle() {
        float k[3] = {Kp, Ki, Kd};
        float bestError = testRoutine();

        for (int i = 0; i < 3; i++) {
            // Serial.print("Tuning parameter ");
            Serial.print(i);
            k[i] += dk[i];
            setGains(k[0], k[1], k[2]);
            float err = testRoutine();

            if (err < bestError) {
                bestError = err;
                dk[i] *= (1 + ksi);
            } else {
                k[i] -= 2 * dk[i];
                setGains(k[0], k[1], k[2]);
                err = testRoutine();

                if (err < bestError) {
                    bestError = err;
                    dk[i] *= (1 + ksi);
                } else {
                    k[i] += dk[i]; // restore
                    dk[i] *= (1 - ksi);
                }
            }
        }

        // Keep running with tuned gains and send results
        while (true) {
            update(0.0f, 0.0f);
            AdaptiveController::setReferences(
                PIDController::getOmegaLeft(),
                PIDController::getOmegaRight()
            );
            AdaptiveController::update();

            // Send tuned gains as feedback packet
            Wifi::sendFeedback(
                Kd,        // use "v" field to carry Kp
                0.0f,        // use "w" field to carry Ki
                Kp,        // use "v_ref" field to carry Kd
                Ki,      // w_ref
                0.0f, 0.0f, // u_L, u_R
                0.0f, 0.0f, // omega_L, omega_R
                0.0f, 0.0f, // w_L, w_R
                0.0f, 0.0f, // theta1_L, theta2_L
                0.0f, 0.0f, // theta1_R, theta2_R
                0.0f, 0.0f  // e_L, e_R
            );

            delay(500);
        }
    }

    // === Test routine (robot moves in a square trajectory) ===
    float testRoutine() {
        float v_ref = 0.0f;
        float w_ref = 0.0f;
        float accumulatedError = 0.0f;
        int steps = 0;

        static unsigned long previous_t;
        unsigned long t;

        // === Square forward ===
        for (int i = 0; i < 4; i++) {
            // Move forward
            previous_t = millis();
            while ((t = millis()) - previous_t < 350) {
                v_ref = 0.5f; w_ref = 0.0f;
                float w_meas = IMU::get_w();
                accumulatedError += fabs(w_ref - w_meas);
                steps++;
                update(v_ref, w_ref);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();
            }
            // Turn right
            previous_t = millis();
            while ((t = millis()) - previous_t < 314) {
                v_ref = 0.0f; w_ref = 5.0f;
                float w_meas = IMU::get_w();
                accumulatedError += fabs(w_ref - w_meas);
                steps++;
                update(v_ref, w_ref);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();
            }
        }

        // Stop
        previous_t = millis();
        while ((t = millis()) - previous_t < 300) {
            v_ref = 0.0f; w_ref = 0.0f;
            float w_meas = IMU::get_w();
            accumulatedError += fabs(w_ref - w_meas);
            steps++;
            update(v_ref, w_ref);
            AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
            AdaptiveController::update();
        }

        // === Square backward ===
        for (int i = 0; i < 4; i++) {
            // Move backward
            previous_t = millis();
            while ((t = millis()) - previous_t < 350) {
                v_ref = -0.5f; w_ref = 0.0f;
                float w_meas = IMU::get_w();
                accumulatedError += fabs(w_ref - w_meas);
                steps++;
                update(v_ref, w_ref);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();
            }
            // Turn left
            previous_t = millis();
            while ((t = millis()) - previous_t < 314) {
                v_ref = 0.0f; w_ref = -5.0f;
                float w_meas = IMU::get_w();
                accumulatedError += fabs(w_ref - w_meas);
                steps++;
                update(v_ref, w_ref);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();
            }
            // Stop
            previous_t = millis();
            while ((t = millis()) - previous_t < 300) {
                v_ref = 0.0f; w_ref = 0.0f;
                float w_meas = IMU::get_w();
                accumulatedError += fabs(w_ref - w_meas);
                steps++;
                update(v_ref, w_ref);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();
            }
        }

        Motor::stop();
        return (steps > 0) ? (accumulatedError / steps) : 1e9f; // return mean error
    }   
    float getOmegaLeft()  { return omega_L_ref; }
    float getOmegaRight() { return omega_R_ref; }

}
