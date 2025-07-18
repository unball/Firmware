#include "adaptive_controller.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"

const float pwm_max = 212.0f;
const float R = 0.021f;
const float max_safe_pwm = 50.0f;

namespace AdaptiveController {

    // === Parameters ===
    const float T = 0.01f;             // Sampling time [s]
    const float tau_m = 0.01f;         // Reference model time constant [s]
    const float gamma_adapt = 0.05f;   // Adaptation gain

    const float am = exp(-T / tau_m);   
    const float bm = 1.0f - am;         

    const float theta1_max = 25.0f;     
    const float theta1_min = -25.0f;
    const float theta2_max = 3.6f;
    const float theta2_min = -3.6f;

    const float sigma = 0.05f;  // E-modification gain

    const float motor_deadzone_c = 20.0f;   // Minimum PWM value to move the motor
    const float deadzone_threshold = 0.15f; // Threshold for adaptation to kick in

    float theta1_L = (pwm_max * R)/v_max, theta2_L = 0.0f;
    float theta1_R = (pwm_max * R)/v_max, theta2_R = 0.0f;

    float omega_L = 0.0f, omega_R = 0.0f;
    float omega_m_L = 0.0f, omega_m_R = 0.0f;
    float u_L = 0.0f, u_R = 0.0f;

    float r_L = 0.0f, r_R = 0.0f;

    unsigned long last_time = 0;

    float applyDeadzone(float u_in, float dz_pos, float dz_neg) {
        if (u_in > 0.0f)
            return (u_in > dz_pos) ? u_in : dz_pos;
        else if (u_in < 0.0f)
            return (u_in < -dz_neg) ? u_in : -dz_neg;
        return 0.0f;
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

        // === RIGHT WHEEL ===
        // Reference model
        omega_m_R = am * omega_m_R + bm * r_R;

        // Compute tracking error
        float e_R = omega_R - omega_m_R;

        // Apply deadzone method: only adapt if |e_R| > threshold
        if (fabs(e_R) > deadzone_threshold) {
            float delta_theta1_R = -T * (gamma_adapt * r_R * e_R - sigma * fabs(e_R) * theta1_R);
            float delta_theta2_R =  T * (gamma_adapt * omega_R * e_R - sigma * fabs(e_R) * theta2_R);
            
            // Projection method for theta1: clamp within limits
            if (!((theta1_R >= theta1_max && delta_theta1_R > 0) || (theta1_R <= theta1_min && delta_theta1_R < 0)))
            theta1_R += delta_theta1_R;
            
            // Projection method for theta2: clamp within limits
            if (!((theta2_R >= theta2_max && delta_theta2_R > 0) || (theta2_R <= theta2_min && delta_theta2_R < 0)))
                theta2_R += delta_theta2_R;
        }


        // Control law
        u_R = theta1_R * r_R - theta2_R * omega_R;
        
        // Constrain control signal to safe PWM limits and sign based on reference
        u_R = constrain(u_R, (r_R >= 0 ? 0.0f : -max_safe_pwm), (r_R >= 0 ? max_safe_pwm : 0.0f));

        float u_R_adj = applyDeadzone(u_R, motor_deadzone_c, motor_deadzone_c);
        Motor::move(MOTOR_RIGHT, u_R_adj);

        // === LEFT WHEEL ===
        // Reference model
        omega_m_L = am * omega_m_L + bm * r_L;
        float e_L = omega_L - omega_m_L;

        // Apply deadzone method: only adapt if |e_L| > threshold
        if (fabs(e_L) > deadzone_threshold) {
            float delta_theta1_L = -T * (gamma_adapt * r_L * e_L - sigma * fabs(e_L) * theta1_L);
            float delta_theta2_L =  T * (gamma_adapt * omega_L * e_L - sigma * fabs(e_L) * theta2_L);
            
            // Projection method for theta1: clamp within limits
            if (!((theta1_L >= theta1_max && delta_theta1_L > 0) || (theta1_L <= theta1_min && delta_theta1_L < 0)))
            theta1_L += delta_theta1_L;
            
            // Projection method for theta2: clamp within limits
            if (!((theta2_L >= theta2_max && delta_theta2_L > 0) || (theta2_L <= theta2_min && delta_theta2_L < 0)))
                theta2_L += delta_theta2_L;
        }


        // Control law
        u_L = theta1_L * r_L - theta2_L * omega_L;

        // Constrain control signal to safe PWM limits and sign based on reference
        u_L = constrain(u_L, (r_L >= 0 ? 0.0f : -max_safe_pwm), (r_L >= 0 ? max_safe_pwm : 0.0f));

        float u_L_adj = applyDeadzone(u_L, motor_deadzone_c, motor_deadzone_c);
        Motor::move(MOTOR_LEFT, u_L_adj);
    }

    void bypassControl() {
        // Apply direct PWM from reference (feedforward only)
        float pwm_R = r_R * pwm_max * R / 1.0f;  // Arbitrary scaling
        float pwm_L = r_L * pwm_max * R / 1.0f;

        pwm_R = applyDeadzone(constrain(pwm_R, -max_safe_pwm, max_safe_pwm), motor_deadzone_c, motor_deadzone_c);
        pwm_L = applyDeadzone(constrain(pwm_L, -max_safe_pwm, max_safe_pwm), motor_deadzone_c, motor_deadzone_c);

        Motor::move(MOTOR_RIGHT, pwm_R);
        Motor::move(MOTOR_LEFT, pwm_L);
        return;
    }

    float getTheta1Left()  { return theta1_L; }
    float getTheta2Left()  { return theta2_L; }
    float getTheta1Right() { return theta1_R; }
    float getTheta2Right() { return theta2_R; }

    float getOmegaLeft()  { return omega_L; }
    float getOmegaRight() { return omega_R; }

    float getControlSignalLeft()  { return u_L; }
    float getControlSignalRight() { return u_R; }
}
