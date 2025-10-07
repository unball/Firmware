#include "pid_controller.hpp"
#include "wifi.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "..\..\include\config.h"
#include <Arduino.h>
#include "adaptive_controller.h"    


namespace PIDController {

    // === Twiddle parameters ===
    // Better starting steps (≈ 20–25% of K)
    static float dk[3] = {0.25f, 0.03f, 0.002f};  // for Kp=1.02, Ki=0.10, Kd=0.008
    static float ksi = 0.2f;                    // learning rate
    static float epsilon = 1e-2f;               // stop threshold
    static int maxIterations = 30;              // safety limit

    // === Internal variable for error tracking ===
    static float maxError = 0.0f;


    // ---- Tunables for the test routine ----
    static constexpr float T_FORWARD_S     = 0.70f;   // straight-line segment duration [s]
    static constexpr float T_HOLD_S        = 0.25f;   // zero-hold between blocks [s]
    static constexpr float T_TURN_S        = 0.30f;   // step turn duration [s]
    static constexpr int   TURN_CYCLES     = 2;       // number of +w/-w pairs
    static constexpr float V_STRAIGHT      = 0.25f;   // m/s while testing straight line
    static constexpr float W_STEP          = 4.0f;    // rad/s step amplitude for rotation tests

    // Weights to blend rotation and straight-line performance into one scalar cost
    static constexpr float W_ROT_COST      = 0.5f;    // weight for rotation error
    static constexpr float W_STRAIGHT_COST = 0.5f;    // weight for straight-line error

    // Utility to convert seconds to milliseconds
    static inline unsigned long sec_to_ms(float s) { return (unsigned long)(s * 1000.0f); }

    // === PID gains ===
    static float Kp = 1.02f;
    static float Ki = 0.1f;
    static float Kd = 0.008f;
    

    // === Robot parameters ===
    static constexpr float T = 0.02f;     // Sampling period [s]

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
        // omega_L_ref = 0.0f;
        // omega_R_ref = 0.0f;
        // lastTime = millis();
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
        
        if (v_ref == 0 && w_ref == 0) {
            Motor::stop();
            return;
        }

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

    float testRoutine_circleDual() {
        // === Test parameters ===
        const float v_cmd = 0.25f;     // linear velocity [m/s]
        const float w_cmd = 2.5f;      // angular velocity [rad/s]
        const float T_SEG  = (2.0f * M_PI) / w_cmd;  // time to complete one full turn (~2.5s)
        const float T_HOLD = 0.3f;     // pause between directions [s]

        // Accumulators
        float err_sum_CW  = 0.0f;
        float err_sum_CCW = 0.0f;
        int   steps_CW = 0;
        int   steps_CCW = 0;

        unsigned long last_loop = millis();
        // Serial.println("[TEST] Running dual circular motion routine...");

        // -------------------------
        // === CLOCKWISE CIRCLE ===
        // -------------------------
        {
            unsigned long t0 = millis();
            unsigned long dt_ms = (unsigned long)(T_SEG * 1000.0f);

            while ((millis() - t0) < dt_ms) {
                unsigned long now = millis();

                if ((now - last_loop) >= (unsigned long)(T * 1000.0f)) {
                    last_loop = now;

                    update(v_cmd, -w_cmd); // negative w = clockwise
                    AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                    AdaptiveController::update();

                    float w_meas = IMU::get_w();
                    err_sum_CW += fabs((-w_cmd) - w_meas);
                    steps_CW++;
                }

            }

            // Motor::stop();
            // Serial.println("[TEST] CW circle done.");
        }
        // vTaskDelay(1);

        // --- short hold between directions ---
        {
            unsigned long t0 = millis();
            while ((millis() - t0) < (unsigned long)(T_HOLD * 1000.0f)) {
                update(0, 0);
            }
        }


        // -------------------------
        // === COUNTER-CLOCKWISE CIRCLE ===
        // -------------------------
        {
            unsigned long t0 = millis();
            unsigned long dt_ms = (unsigned long)(T_SEG * 1000.0f);

            while ((millis() - t0) < dt_ms) {
                unsigned long now = millis();

                if ((now - last_loop) >= (unsigned long)(T * 1000.0f)) {
                    last_loop = now;

                    update(v_cmd, +w_cmd); // positive w = counterclockwise
                    AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                    AdaptiveController::update();

                    float w_meas = IMU::get_w();
                    err_sum_CCW += fabs((+w_cmd) - w_meas);
                    steps_CCW++;
                }

            }

            // update(0, 0);
            // Serial.println("[TEST] CCW circle done.");
        }

        // --- short hold between directions ---
        {
            unsigned long t0 = millis();
            while ((millis() - t0) < (unsigned long)(T_HOLD * 1000.0f)) {
                update(0, 0);
                reset();
            }
        }

        // === Compute metrics ===
        if (steps_CW == 0)  steps_CW  = 1;
        if (steps_CCW == 0) steps_CCW = 1;

        float mean_err_CW  = err_sum_CW  / steps_CW;
        float mean_err_CCW = err_sum_CCW / steps_CCW;

        // Combine errors into one cost
        float symmetry_penalty = fabs(mean_err_CW - mean_err_CCW);
        float total_cost = 0.5f * (mean_err_CW + mean_err_CCW) + 0.5f * symmetry_penalty;

        // Serial.println("[TEST] Dual-circle routine complete:");
        // Serial.print("  CW mean |w_err|  = ");  Serial.println(mean_err_CW, 4);
        // Serial.print("  CCW mean |w_err| = ");  Serial.println(mean_err_CCW, 4);
        // Serial.print("  Symmetry penalty = ");   Serial.println(symmetry_penalty, 4);
        // Serial.print("  Total cost = ");          Serial.println(total_cost, 4);

        return total_cost;
    }

    void run_segment(float seg_seconds, float v_cmd, float w_cmd, bool accumulate_rotation, bool accumulate_straight, float &rot_err_sum, int &rot_steps, float &st_w_sum, int &st_steps) {
        unsigned long t_start = millis();
        unsigned long t_now = t_start;
        unsigned long duration_ms = (unsigned long)(seg_seconds * 1000.0f);
        unsigned long last_update = t_start;

        while ((t_now - t_start) < duration_ms) {
            t_now = millis();

            // Run control only every T = 20 ms
            if ((t_now - last_update) >= (unsigned long)(T * 1000.0f)) {
                last_update = t_now;

                // --- Control update ---
                update(v_cmd, w_cmd);
                AdaptiveController::setReferences(getOmegaLeft(), getOmegaRight());
                AdaptiveController::update();

                // --- Measurement ---
                float w_meas = IMU::get_w();

                if (accumulate_rotation) {
                    rot_err_sum += fabs(w_cmd - w_meas);
                    rot_steps++;
                }

                if (accumulate_straight) {
                    st_w_sum += fabs(w_meas); // should be near zero if straight
                    st_steps++;
                }
            }

            // Safety timeout (prevents infinite loop)
            if ((millis() - t_start) > duration_ms * 2) { break; }
        }
    }

    void twiddle() {
        float k[3] = {Kp, Ki, Kd};
        float bestError = testRoutine_circleDual();

        for (int iter = 0; iter < maxIterations; iter++) {
            for (int i = 0; i < 3; i++) {
                k[i] += dk[i];
                setGains(k[0], k[1], k[2]);
                float err = testRoutine_circleDual();

                if (err < bestError) {
                    bestError = err;
                    dk[i] *= (1 + ksi);
                } else {
                    k[i] -= 2 * dk[i];
                    setGains(k[0], k[1], k[2]);
                    err = testRoutine_circleDual();

                    if (err < bestError) {
                        bestError = err;
                        dk[i] *= (1 + ksi);
                    } else {
                        k[i] += dk[i]; // restore
                        dk[i] *= (1 - ksi);
                    }
                }
            }

            k[0] = constrain(k[0], 0.0f, 5.0f);   // Kp
            k[1] = constrain(k[1], 0.0f, 5.0f);   // Ki
            k[2] = constrain(k[2], 0.0f, 1.0f);  // Kd

            // Apply the corrected gains
            setGains(k[0], k[1], k[2]);

            Wifi::sendFeedback(
                Kd,        // use "v" field to carry Kp
                0.0f,        // use "w" field to carry Ki
                Kp,        // use "v_ref" field to carry Kp
                Ki,      // w_ref
                dk[0], dk[1], // u_L, u_R
                bestError, 0.0f, // omega_L, omega_R
                dk[2], 0.0f, // w_L, w_R
                0.0f, 0.0f, // theta1_L, theta2_L
                0.0f, 0.0f, // theta1_R, theta2_R
                0.0f, 0.0f  // e_L, e_R
            );

            // === Stop if improvements are very small ===
            float sumDk = dk[0] + dk[1] + dk[2];
            if (sumDk < epsilon) break;
        }
    }

    float testRoutine() {
        float rot_err_sum = 0.0f;
        int   rot_steps   = 0;
        float st_w_sum    = 0.0f;
        int   st_steps    = 0;

        // ---- Phase A: Rotation tests ----
        for (int k = 0; k < TURN_CYCLES; k++) {
            run_segment(T_TURN_S,  0.0f, +W_STEP, true,  false,
                        rot_err_sum, rot_steps, st_w_sum, st_steps);
            run_segment(T_HOLD_S,  0.0f, 0.0f,    true,  false,
                        rot_err_sum, rot_steps, st_w_sum, st_steps);
            run_segment(T_TURN_S,  0.0f, -W_STEP, true,  false,
                        rot_err_sum, rot_steps, st_w_sum, st_steps);
            run_segment(T_HOLD_S,  0.0f, 0.0f,    true,  false,
                        rot_err_sum, rot_steps, st_w_sum, st_steps);
        }

        // ---- Phase B: Straight-line tests ----
        run_segment(T_FORWARD_S,  V_STRAIGHT,  0.0f, false, true,
                    rot_err_sum, rot_steps, st_w_sum, st_steps);
        run_segment(T_HOLD_S,     0.0f,        0.0f, false, true,
                    rot_err_sum, rot_steps, st_w_sum, st_steps);
        run_segment(T_FORWARD_S, -V_STRAIGHT,  0.0f, false, true,
                    rot_err_sum, rot_steps, st_w_sum, st_steps);
        run_segment(T_HOLD_S,     0.0f,        0.0f, false, true,
                    rot_err_sum, rot_steps, st_w_sum, st_steps);

        update(0, 0);

        if (rot_steps == 0) rot_steps = 1;
        if (st_steps  == 0) st_steps  = 1;

        float rot_cost      = rot_err_sum / rot_steps;
        float straight_cost = st_w_sum / st_steps;
        float total_cost = W_ROT_COST * rot_cost + W_STRAIGHT_COST * straight_cost;

        return total_cost;
    }





    float getOmegaLeft()  { return omega_L_ref; }
    float getOmegaRight() { return omega_R_ref; }

}
