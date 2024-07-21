#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorA;
        float motorB;
    }vel;
    
    void setup();
    void add_A();
    void add_B();
    void resetEncoders();
    double time_counter();
    vel encoder();
    double get_ticks_per_ms();

}
#endif