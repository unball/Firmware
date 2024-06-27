#ifndef CONFIG_HPP
#define CONFIG_HPP

/* Main configs*/
#define WEMOS_DEBUG false 
#define CONTROL_TESTER false
#define TWIDDLE false
#define ROBOT_NUMBER 0

/* Wi-Fi */ 
const int communicationTimeout = 500000;
const int resetTimeout = 4000000;
const float MAX_POWER = 10.0;

/* Control */
const int controlLoopInterval= 200;   // actuation interval in ms

/* Robot parameters */
const float r =	0.01525;
const float L = 0.0605;
const int motor_deadzone = 32;

#endif
