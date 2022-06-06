#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <radio.hpp>
#include <waves.hpp>

//#if TEENSY_DEBUG
//#define CONTROL_DEBUG true
//#else
//#define CONTROL_DEBUG false
//#endif

typedef struct{
    double lin;
    double ang;
} linAng;

namespace Control{
    void stand();
}

#endif