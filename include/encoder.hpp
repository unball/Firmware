#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "pins.h"
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorA;
        float motorB;
    }vel;
    
    void setup();
    void somaA();
    void somaB();
    void resetEncoders();
    int32_t timeCounter();
    vel encoder();
}
#endif