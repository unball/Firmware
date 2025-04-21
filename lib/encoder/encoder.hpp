#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorLeft;
        float motorRight;
    }vel;
    
    void setup();
    vel getMotorSpeeds();

}
#endif