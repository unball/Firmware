#ifndef CONFIG_HPP
#define CONFIG_HPP

/* Main configs*/
#define WEMOS_DEBUG false 
#define CONTROL_TESTER false
#define DEAD_ZONE_TESTER false
#define TWIDDLE false
#define ROBOT_NUMBER 0

/* Wi-Fi */ 
const int communicationTimeout = 500000;
const int resetTimeout = 4000000;
const float MAX_POWER = 10.0;


/* Control */
const int controlLoopInterval= 200;   // actuation interval in ms
const double Kv = 1;
const double Kw = 1;


/*Sensor*/
const double TICKS2METER = 2*PI*0.03*1000/(48); // converts ticks/ms into m/s



/* Robot parameters */
const float r = 0.042;
const float L = 0.101;  
const int motor_deadzone = 2;
const float v_max = 2;
const float w_max = 65;
const double pwm_max = 212;



#endif
