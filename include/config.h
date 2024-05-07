#ifndef CONFIG_HPP
#define CONFIG_HPP

/* Main configs*/
#define WEMOS_DEBUG true
#define ROBOT_NUMBER 0

/* Wi-Fi */
const int communicationTimeout = 500000;
const int resetTimeout = 4000000;
const float MAX_POWER = 10.0;

/* Control */
const int controlLoopInterval= 2000;   // actuation interval in ms
const float kp = 0.159521;
const float ki = 0.016864;
const float kd = 0.016686;

/* Robot parameters */
const float r =	0.016;
const float L = 0.075;
const int motor_deadzone = 32;


#endif
