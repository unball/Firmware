#ifndef RADIO_HPP
#define RADIO_HPP
#include <Arduino.h>
#include <RF24.h>
#include <stdint-gcc.h>
#include "pins.h"

namespace Radio{

    struct dataStruct{
        int16_t A[5];
        int16_t B[5];
    };

    struct vels{
        int16_t A;
        int16_t B;
    };

    extern RF24 radio;

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(vels *);
    void reportMessage(int);
}




#endif