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
    void calc();
    void calc2();
    void resetEncoders();
    uint32_t time_counter();
    // vel encoder();
    double get_ticks_per_us();
    // double get_w();

}
#endif