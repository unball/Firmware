#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <imu.hpp>
#include <waves.hpp>
#include "wifi.hpp"
#include <encoder.hpp>
#include "../../include/config.h"

namespace Control{
    int32_t deadzone(int32_t vin, int32_t up, int32_t down);
    void readSpeeds(double *w, double *v);
    double linSpeed(Encoder::vel enc);
    double PID(double v, double err);
    void control(double v, double w, double currW, double currV);
    void speed2motors(double v, double w);
    void stand();
    struct Control_cee{
        double v;
        double w;
    };
    
}

#endif