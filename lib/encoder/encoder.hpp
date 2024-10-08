#ifndef ENCODER_HPP
#define ENCODER_HPP
#include "motor.hpp"

namespace Encoder{

    typedef struct{
        float motorA;
        float motorB;
    }vel;
    
    void setup();
    void somaMotorAChanelA();
    void somaMotorBChanelA();
    void somaMotorAChanelB();
    void somaMotorBChanelB();
    float calibration_encoder(float v);
    void resetEncoders();
    double timeCounter();
    vel encoder();
}
#endif