#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <imu.hpp>
#include <waves.hpp>
#include "wifi.hpp"

namespace Control{
	const float kp = -1.59521;
	const float ki = -0.16864;
	const float kd = 0.16686;

    void stand();
}

#endif