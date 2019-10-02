#ifndef RADIO_HPP
#define RADIO_HPP
#include <Arduino.h>
#include <RF24.h>
#include <stdint-gcc.h>
#include "pins.h"

namespace Radio{

    struct dataStruct{
        int32_t A;
        int32_t B;
        double Kp[2], Ki[2], Kd[2];
    };

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(dataStruct*);
    void reportMessage(int);
}




#endif