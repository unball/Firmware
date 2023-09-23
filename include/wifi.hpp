#ifndef WIFI_H
#define WIFI_H
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "pins.h"

namespace Wifi{

    const float MAX_POWER = 10.5;

    typedef struct dataStruct
    {
        uint8_t id;
        double v;
        double w;
    } dataStruct;

    void setup(uint8_t robot);
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
    void receiveData(double *v, double *w);
    bool isCommunicationLost();

}

#endif