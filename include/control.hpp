#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <radio.hpp>
#include <imu.hpp>
#include <waves.hpp>

typedef struct{
    double lin;
    double ang;
} linAng;

namespace Control{
    void stand();
}

#endif