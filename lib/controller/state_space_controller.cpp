#include "state_space_controller.hpp"
#include "encoder.hpp"
#include "IMU.hpp"
#include "config.h"

const float R = 0.02f; // Wheel radius in meters
const float L = 0.075f; // Distance between wheels in meters

namespace StateSpaceController {

    const float K[2][2] = {{1.0f, 0.0f},
                           {0.0f, 3.0f}};
    const float A[2][2] = {{1.0f, 0.0f},
                           {0.0f, 1.0f}};
    const float B[2][2] = {{1.0f, 0.0f},
                           {0.0f, 1.0f}};
    const float C[2][2] = {
                           {1.0f / R,  (R * L) / 2.0f},
                           {1.0f / R, -(R * L) / 2.0f}};

    float omega_ref_left = 0.0f;
    float omega_ref_right = 0.0f;


    void update(float v_ref, float w_ref) {
        Encoder::vel vel = Encoder::getMotorSpeeds();
        float omega_L = vel.motorLeft;
        float omega_R = vel.motorRight;
        float v = (R / 2.0f) * (omega_R + omega_L);
        float w = IMU::get_w();

        float x[2] = {v, w};
        float r[2] = {v_ref, w_ref};
        float e[2] = {r[0] - x[0], r[1] - x[1]};

        float u[2];
        matrix_multiply(K, e, u);

        float Ax[2], Bu[2], x_next[2];
        matrix_multiply(A, x, Ax);
        matrix_multiply(B, u, Bu);
        vector_add(Ax, Bu, x_next);

        float y[2];
        matrix_multiply(C, x_next, y);

        omega_ref_right = y[0];
        omega_ref_left = y[1];

        // Debug print
        printAlignedFloat(millis() / 1000.0f, 5, 2); Serial.print(" | [REF] v:"); printAlignedFloat(v_ref, 5, 2);
        Serial.print(" | w:"); printAlignedFloat(w_ref, 5, 2);
        Serial.print(" || [MEAS] v:"); printAlignedFloat(v, 5, 2);
        Serial.print(" | w:"); printAlignedFloat(w, 5, 2);
        Serial.print(" || [e_v, e_w]:"); printAlignedFloat(e[0], 6, 2); Serial.print(","); printAlignedFloat(e[1], 6, 2);
        Serial.print(" || [u] v_d:"); printAlignedFloat(u[0], 6, 2); Serial.print(" | w_d:"); printAlignedFloat(u[1], 6, 2);
        Serial.print(" || [ωL_r, ωR_r]:"); printAlignedFloat(omega_ref_left, 6, 2); Serial.print(","); printAlignedFloat(omega_ref_right, 6, 2);
        Serial.print(" || [ωL_meas, ωR_meas]:"); printAlignedFloat(omega_L, 6, 2); Serial.print(","); printAlignedFloat(omega_R, 6, 2);Serial.println();

    }

    float getOmegaRefLeft() { return omega_ref_left; }
    float getOmegaRefRight() { return omega_ref_right; }
}
