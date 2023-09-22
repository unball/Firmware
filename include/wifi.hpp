#ifndef WIFI_H
#define WIFI_H
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "pins.h"

namespace Wifi{

    typedef struct dataStruct
    {
        int16_t id;
        int16_t v;
        int16_t w;
    } dataStruct;

    void setup(uint8_t robot);
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
    void receiveData(int16_t *v, int16_t *w);
    bool isCommunicationLost();

}

#endif