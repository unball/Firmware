// File: state_space_controller.h

#ifndef STATE_SPACE_CONTROLLER_H
#define STATE_SPACE_CONTROLLER_H

#include <Arduino.h>

class StateSpaceController {
  public:
    StateSpaceController(float K11, float K12, float K21, float K22, float R, float L);

    void update(float v_ref, float w_ref);

    float getOmegaRefLeft() const;
    float getOmegaRefRight() const;

  private:
    float K[2][2]; // Control gain matrix
    float wheel_radius;
    float wheel_base;
    unsigned long last_update_us;
    const unsigned long Ts_us = 20000; // 20 ms

    float omega_L_d = 0.0f;
    float omega_R_d = 0.0f;

    void computeControl(float v_ref, float w_ref);
};

#endif
