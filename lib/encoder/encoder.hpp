#ifndef ENCODER_HPP
#define ENCODER_HPP
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
    double timeCounter();
    vel encoder();
}
#endif