#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <stdint-gcc.h>
#include "pins.h"
#if TENSY_DEBUG
#define MOTOR_DEBUG true
#else
#define MOTOR_DEBUG false
#endif

namespace Motor{

    void setup(void);
    void move(uint8_t motor, int32_t power);
    void stop();
    int8_t getMotorDirection(int8_t motor);
}





#endif