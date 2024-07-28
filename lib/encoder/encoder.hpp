#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorA;
        float motorB;
    }vel;
    
    void setup();
    void IRAM_ATTR calc();
    double get_w();

}
#endif