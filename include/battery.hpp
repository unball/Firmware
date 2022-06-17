#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <Arduino.h>
#include "pins.h"

#define MAX_VOLTAGE 7.4

namespace Battery{

    float map_float(float x, float in_min, float in_max, float out_min, float out_max);
    void measure(float *voltage);
}

#endif