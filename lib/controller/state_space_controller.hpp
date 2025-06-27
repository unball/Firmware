#ifndef STATE_SPACE_CONTROLLER_HPP
#define STATE_SPACE_CONTROLLER_HPP

#include <Arduino.h>

namespace StateSpaceController {
    // Atualiza o controlador com referência de velocidade linear e angular
    void update(float v_ref, float w_ref);

    inline void matrix_multiply(const float mat[2][2], const float vec[2], float result[2]) {
      result[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1];
      result[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1];
    }

    inline void vector_add(const float vec1[2], const float vec2[2], float result[2]) {
      result[0] = vec1[0] + vec2[0];
      result[1] = vec1[1] + vec2[1];
    }

    inline void printAlignedFloat(float value, int width, int decimals) {
      char buffer[20];
      dtostrf(value, width, decimals, buffer);
      Serial.print(buffer);
    }

    // Obtém as referências de velocidade angular para as rodas esquerda e direita
    float getOmegaRefLeft();
    float getOmegaRefRight();

}

#endif
