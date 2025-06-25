#include "adaptive_controller.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"

// TEMP: teste para validar escopo
const float pwm_max = 212.0f;
const float R = 0.021f;

namespace AdaptiveController {

    const float T = 0.02f;
    const float tau_m = 0.01f;
    const float gamma_adapt = 0.05f;

    const float am = exp(-T / tau_m);
    const float bm = 1.0f - am;

    const float theta1_max = 25.0f;
    const float theta1_min = -25.0f;
    const float theta2_max = 3.5f;
    const float theta2_min = -3.5f;

    const float motor_deadzone_c = 20.0f;
    const float deadzone_threshold = 0.15f;

    float theta1_L = 0.0f;
    float theta2_L = 0.0f;
    float theta1_R = 0.0f;
    float theta2_R = 0.0f;

    float omega_L = 0.0f, omega_R = 0.0f;
    float omega_m_L = 0.0f, omega_m_R = 0.0f;
    float u_L = 0.0f, u_R = 0.0f;

    float r_L = 0.0f;
    float r_R = 0.0f;

    unsigned long last_time = 0;

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

    void setup() {
        theta1_L = (pwm_max * R) / 1.0f;
        theta1_R = (pwm_max * R) / 1.0f;
    }

    void setReferences(float omega_L_ref, float omega_R_ref) {
        r_L = omega_L_ref;
        r_R = omega_R_ref;
    }

    void update() {
        unsigned long now = millis();
        if (now - last_time < (unsigned long)(T * 1000)) return;
        last_time = now;

        Encoder::vel vel = Encoder::getMotorSpeeds();
        omega_L = vel.motorLeft;
        omega_R = vel.motorRight;

        // Right wheel
        omega_m_R = am * omega_m_R + bm * r_R;
        float e_R = omega_R - omega_m_R;
        float e_eff_R = applyDeadzoneError(e_R, deadzone_threshold);
        float u_R_unsat = theta1_R * r_R - theta2_R * omega_R;
        u_R = constrain(u_R_unsat, (r_R >= 0 ? 0.0f : -100.0f), (r_R >= 0 ? 100.0f : 0.0f));

        float delta_theta1_R = -T * gamma_adapt * r_R * e_eff_R;
        float delta_theta2_R =  T * gamma_adapt * omega_R * e_eff_R;

        if ((theta1_R >= theta1_max && delta_theta1_R > 0) || (theta1_R <= theta1_min && delta_theta1_R < 0)) delta_theta1_R = 0;
        if ((theta2_R >= theta2_max && delta_theta2_R > 0) || (theta2_R <= theta2_min && delta_theta2_R < 0)) delta_theta2_R = 0;

        theta1_R += delta_theta1_R;
        theta2_R += delta_theta2_R;

        u_R = theta1_R * r_R - theta2_R * omega_R;
        float u_R_adj = applyDeadzone(u_R, motor_deadzone_c, motor_deadzone_c);
        Motor::move(MOTOR_RIGHT, u_R_adj);

        // Left wheel
        omega_m_L = am * omega_m_L + bm * r_L;
        float e_L = omega_L - omega_m_L;
        float e_eff_L = applyDeadzoneError(e_L, deadzone_threshold);
        float u_L_unsat = theta1_L * r_L - theta2_L * omega_L;
        u_L = constrain(u_L_unsat, (r_L >= 0 ? 0.0f : -100.0f), (r_L >= 0 ? 100.0f : 0.0f));

        float delta_theta1_L = -T * gamma_adapt * r_L * e_eff_L;
        float delta_theta2_L =  T * gamma_adapt * omega_L * e_eff_L;

        if ((theta1_L >= theta1_max && delta_theta1_L > 0) || (theta1_L <= theta1_min && delta_theta1_L < 0)) delta_theta1_L = 0;
        if ((theta2_L >= theta2_max && delta_theta2_L > 0) || (theta2_L <= theta2_min && delta_theta2_L < 0)) delta_theta2_L = 0;

        theta1_L += delta_theta1_L;
        theta2_L += delta_theta2_L;

        u_L = theta1_L * r_L - theta2_L * omega_L;
        float u_L_adj = applyDeadzone(u_L, motor_deadzone_c, motor_deadzone_c);
        Motor::move(MOTOR_LEFT, u_L_adj);

        // Optional logging
        // Serial.print("Ref L: "); Serial.print(r_L);
        // Serial.print(" | Ref R: "); Serial.print(r_R);
        // Serial.print(" | u_L: "); Serial.print(u_L);
        // Serial.print(" | u_R: "); Serial.println(u_R);
    }

    float getTheta1Left() { return theta1_L; }
    float getTheta2Left() { return theta2_L; }
    float getTheta1Right() { return theta1_R; }
    float getTheta2Right() { return theta2_R; }

    float getOmegaLeft() { return omega_L; }
    float getOmegaRight() { return omega_R; }

    float getControlSignalLeft() { return u_L; }
    float getControlSignalRight() { return u_R; }
}
