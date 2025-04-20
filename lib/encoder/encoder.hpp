#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorA;
        float motorB;
    }vel;
    
    void setup();
    double getMotorSpeeds();

}
#endif