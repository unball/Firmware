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
        int16_t vl;
        int16_t vr;
    };

    extern RF24 radio;

    void setup(uint8_t robot, uint8_t sendChannel);
    bool receiveData(double *vl, double *vr);
    bool isRadioLost();
    bool isRadioDisconnected();
}




#endif