#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <motor.hpp>
#include <encoder.hpp>
#include <radio.hpp>
#include <ledRGB.hpp>
#include <imu.hpp>

#if TEENSY_DEBUG
#define CONTROL_DEBUG true
#else
#define CONTROL_DEBUG false
#endif

#define MOTOR_TEST false    //define se está ou não fazendo o teste nos motores
#define wave 1              // sine = 1 -- square = 2 -- step = 3
//#define PI 3.14159265359

typedef struct{
    double lin;
    double ang;
} linAng;

namespace Control{

    void stopRobot();
    void TimeOfCicle();
    void TestWave(int32_t*, int32_t*);
    bool frame_rate();
    void Turbo(int, int);
    void control(int32_t, int32_t);
    void motorId();
    void stand();
    bool isRadioLost(bool, uint32_t);
}

#endif