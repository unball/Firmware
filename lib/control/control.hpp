#ifndef CONTROL_HPP
#define CONTROL_HPP
#include <motor.hpp>
#include <imu.hpp>
#include <waves.hpp>
#include "wifi.hpp"
#include "../../include/config.h"


namespace Control{
    int32_t deadzone(int32_t vin, int32_t up, int32_t down);
    void readSpeeds(double *w);
    double PID(double v, double err);
    void control(double v, double w, double currW);
    void stand(bool useControl);
    void actuateNoControl();
}

#endif