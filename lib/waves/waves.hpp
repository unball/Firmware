#ifndef WAVES_HPP
#define WAVES_HPP

#include <Arduino.h>

namespace Waves{
    float sine_wave();
    double square_wave(double, uint32_t);
    void triangular_wave(int16_t *, int16_t *, int32_t, int32_t);
    void sine_wave(int16_t *, int16_t *);
    void square_wave(int16_t *, int16_t *, int32_t, uint32_t);
    void step(int16_t *, int16_t *);
}

#endif