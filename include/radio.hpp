#ifndef RADIO_HPP
#define RADIO_HPP
#include <Arduino.h>
#include <RF24.h>
#include <stdint-gcc.h>
#include "pins.h"

namespace Radio{

    struct dataStruct{
        int16_t v[5];
        int16_t w[5];
    };
    
    struct reportStruct{
        uint32_t time;
        float v,w;
        float enca, encb;
    };

    struct vels{
        float v;
        float w;
    };

    extern RF24 radio;

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(vels *);
    void reportMessage(reportStruct *);
}




#endif