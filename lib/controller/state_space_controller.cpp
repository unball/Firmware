#include "state_space_controller.hpp"
#include "encoder.hpp"
#include "IMU.hpp"
#include "motor.hpp"
// #include "config.h"
#include "..\..\include\config.h"
#include "robot_config.hpp"
#include <Arduino.h>
#include <math.h>

// === System parameters ===
const float T = 0.02f;       // Sampling time [s]
const float u_min = -69.0f;  // Control saturation
const float u_max =  69.0f;

// === Gains from MATLAB ===
const float K[2][2] = {
    {126.3576f, 5.1305f},
    {126.3576f, -5.1305f}
};

const float N[2][2] = {
    {238.1948f, -2.4948f},
    {238.1948f,  2.4948f}
};

namespace StateSpaceController {

    static float u_L = 0.0f;
    static float u_R = 0.0f;
    static unsigned long lastTime = 0; // keeps last execution timestamp

    void update(float v_ref, float w_ref) {
        unsigned long now = millis();
        if (now - lastTime < (unsigned long)(T * 1000.0f)) {
            return;
        }
        lastTime = now;

        // === Read measured states ===
        Encoder::vel vel = Encoder::getMotorSpeeds();
        float omega_L = vel.motorLeft;
        float omega_R = vel.motorRight;

        float v = (R / 2.0f) * (omega_R + omega_L);
        // float w = IMU::get_w();
        float w_encoders = Encoder::getAngularVelocity(R, L);
        float w = IMU::get_w_filtered(w_encoders , 0.98f);

        float x[2] = {v, w};
        float r[2] = {v_ref, w_ref};

        // === Compute control: u = -Kx + Nr ===
        float u_unsat[2];
        if (v_ref == 0 && w_ref == 0) {
            u_unsat[0] = 0.0f;
            u_unsat[1] = 0.0f;
            Motor::stop();
        } else {
            for (int i = 0; i < 2; i++) {
                u_unsat[i] = 0.0f;
                for (int j = 0; j < 2; j++) {
                    u_unsat[i] += -K[i][j] * x[j] + N[i][j] * r[j];
                }
            }
        }

        // === Apply saturation ===
        u_L = constrain(u_unsat[0], u_min, u_max);
        u_R = constrain(u_unsat[1], u_min, u_max);

    }

    float getControlLeft()  { return u_L; }
    float getControlRight() { return u_R; }
}
