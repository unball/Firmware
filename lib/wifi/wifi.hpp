#ifndef WIFI_H
#define WIFI_H
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "../../include/config.h"

namespace Wifi{

    struct rcv_message
    {
        uint8_t control;
        uint8_t id;
        int16_t v;
        int16_t w;
    };

    struct rcv_twiddle
    {
        uint8_t id;
        float kp;
        float ki;
        float kd;
    };

    struct snd_message{
    float value;
    };

    void setup(uint8_t robot);
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
    bool receiveData(double *v, double *w);
    bool isCommunicationLost();

}

#endif