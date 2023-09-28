#ifndef CONFIG_HPP
#define CONFIG_HPP

/* Main configs*/
#define WEMOS_DEBUG false
#define ROBOT_NUMBER 1

/* Wi-Fi */
const int communicationTimeout = 500000;
const float MAX_POWER = 10.5;

/* Control */
const float kp = 0.159521;
const float ki = 0.016864;
const float kd = 0.016686;
const float r =	0.016;
const float L = 0.075;


#endif
