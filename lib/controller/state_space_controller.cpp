#include "state_space_controller.hpp"
#include "encoder.hpp"
#include "IMU.hpp"
#include "config.h"
#include <math.h>

const float R = 0.021f;
const float L = 0.075f;
const float T = 0.02f;
const float tau = 0.05f;

namespace StateSpaceController {

    // Ganhos de realimentação de estados K
    const float K[2][2] = {
        {3.0f, 0.0f},
        {0.0f, 3.0f}
    };

    // Ganhos de rastreamento de referência N (calculados no MATLAB)
    const float N[2][2] = {
        {1.1f, 0.0f},
        {0.0f, 1.1f}
    };

    // Matriz de conversão para as velocidades das rodas (C_wheels)
    const float C_wheels[2][2] = {
        {1.0f / R,  (R * L) / 2.0f},
        {1.0f / R, -(R * L) / 2.0f}
    };

    float omega_ref_left = 0.0f;
    float omega_ref_right = 0.0f;

    void update(float v_ref, float w_ref) {
        // === Medição dos estados ===
        Encoder::vel vel = Encoder::getMotorSpeeds();
        float omega_L = vel.motorLeft;
        float omega_R = vel.motorRight;

        float v = (R / 2.0f) * (omega_R + omega_L);
        float w = IMU::get_w();  // velocidade angular do robô

        float x[2] = {v, w};
        float r[2] = {v_ref, w_ref};

        // u = -Kx + Nr
        float Kx[2], Nr[2];
        matrix_multiply(K, x, Kx);  // Kx = K * x
        matrix_multiply(N, r, Nr);  // Nr = N * r

        float minus_Kx[2] = {-Kx[0], -Kx[1]};  // -Kx
        float u[2];
        vector_add(minus_Kx, Nr, u); // u = -Kx + Nr

        // === Conversão para omega_L e omega_R ===
        float y[2];
        matrix_multiply(C_wheels, u, y); // y = C_wheels * u

        omega_ref_right = y[0];
        omega_ref_left = y[1];

        // === Debug print ===
        printAlignedFloat(millis() / 1000.0f, 5, 2); Serial.print(" | [REF] v:"); printAlignedFloat(v_ref, 5, 2);
        Serial.print(" | w:"); printAlignedFloat(w_ref, 5, 2);
        Serial.print(" || [MEAS] v:"); printAlignedFloat(v, 5, 2);
        Serial.print(" | w:"); printAlignedFloat(w, 5, 2);
        Serial.print(" || [u] v_d:"); printAlignedFloat(u[0], 6, 2); Serial.print(" | w_d:"); printAlignedFloat(u[1], 6, 2);
        Serial.print(" || [ωL_r, ωR_r]:"); printAlignedFloat(omega_ref_left, 6, 2); Serial.print(","); printAlignedFloat(omega_ref_right, 6, 2);
        Serial.print(" || [ωL_meas, ωR_meas]:"); printAlignedFloat(omega_L, 6, 2); Serial.print(","); printAlignedFloat(omega_R, 6, 2);
        Serial.println();
    }

    float getOmegaRefLeft() { return omega_ref_left; }
    float getOmegaRefRight() { return omega_ref_right; }
}
