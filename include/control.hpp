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

/* Definições para o modo de identificação */
#define CONTROL_ID false

// Modo de transferência dos dados
#define CONTROL_ID_TRANSFER CONTROL_ID_TRANSFER_NONE
#define CONTROL_ID_TRANSFER_NONE   0
#define CONTROL_ID_TRANSFER_RADIO  1
#define CONTROL_ID_TRANSFER_SERIAL 2

// Etapa da identificação
#define CONTROL_ID_MODE CONTROL_ID_MODE_VALIDATION
#define CONTROL_ID_MODE_DEADZONE    1
#define CONTROL_ID_MODE_ID          2
#define CONTROL_ID_MODE_VALIDATION  3

// Tamanho do buffer de velocidades a serem transmitidas
#if (CONTROL_ID_MODE == CONTROL_ID_MODE_DEADZONE)
    #define CONTROL_ID_BUFFER_SIZE 3500
#elif (CONTROL_ID_MODE == CONTROL_ID_MODE_ID)
    #define CONTROL_ID_BUFFER_SIZE 1000
#elif (CONTROL_ID_MODE == CONTROL_ID_MODE_VALIDATION)
    #define CONTROL_ID_BUFFER_SIZE 1000
#endif


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
    void control(double, double);
    void motorId();
    void stand();
    bool isRadioLost(bool, uint32_t);
}

#endif