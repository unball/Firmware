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
    
    /**
     * @brief Calculate robot's angular velocity from wheel speeds
     * @param R Wheel radius in meters (default from config.h)
     * @param L Distance between wheels in meters (default from config.h)
     * @return Angular velocity in rad/s
     */
    float getAngularVelocity(float R = 0.0215f, float L = 0.0825f);

}
#endif