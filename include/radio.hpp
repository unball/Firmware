#ifndef RADIO_HPP
#define RADIO_HPP
#include <Arduino.h>
#include <RF24.h>
#include <stdint-gcc.h>
#include "pins.h"

#define NUMBER_OF_ROBOTS 5

namespace Radio{

    struct dataStruct{
        int32_t A;
        int32_t B;
        double Kp[2], Ki[2], Kd[2];
    };

    typedef struct {
        float vel_A;
        float vel_B;
        int32_t in_A, in_B;
        uint32_t time;
    } vel;

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(dataStruct*);
    void reportMessage(int);
    void reportMessage(vel);
}




#endif