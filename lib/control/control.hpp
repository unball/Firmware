#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <imu.hpp>
#include <waves.hpp>
#include "wifi.hpp"

namespace Control{
	const float kp = 0.159521;
	const float ki = 0.016864;
	const float kd = 0.016686;
	const float r =	0.016;
	const float L = 0.075;

    void stand();
}

#endif