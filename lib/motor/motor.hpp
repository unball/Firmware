#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include "../../include/pins.h"

enum MotorId {
    MOTOR_RIGHT = 0,    // Motor B
    MOTOR_LEFT = 1      // Motor A
};

enum Direction {
    FORWARD = 1,
    BACKWARD = -1
};

namespace Motor {
    void setup(void);
    void move(uint8_t motor, int32_t power);
    void stop();
    int8_t getMotorDirection(int8_t motor);
}

#endif
