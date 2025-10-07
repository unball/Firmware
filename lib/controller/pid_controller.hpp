#pragma once

namespace PIDController {
    void reset();
    void setGains(float kp, float ki, float kd);
    void update(float v_ref, float w_ref);  // Update PID for angular velocity and compute wheel refs
    void twiddle(); // Auto-tune PID gains using Twiddle algorithm
    float testRoutine(); // Run a test routine to evaluate performance
    float testRoutine_circleDual();
    float getOmegaLeft();
    float getOmegaRight();
}
