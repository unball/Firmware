#ifndef RADIO_HPP
#define RADIO_HPP

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <stdint-gcc.h>
#include "pins.h"

#define RADIO_THRESHOLD 2000000
#define RADIO_RESET_THRESHOLD 4000000

namespace Radio{

    struct dataStruct{
        float v;
        float w;
    };

    extern RF24 radio;

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(double *v, double *w);
    bool isRadioLost();
    bool isRadioDisconnected();
}




#endif