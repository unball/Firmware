#pragma once

namespace PIDController {
    void reset();
    void setGains(float kp, float ki, float kd);
    void update(float v_ref, float w_ref);  // Update PID for angular velocity and compute wheel refs
    float getOmegaLeft();
    float getOmegaRight();
}
