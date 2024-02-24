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
        int16_t kp;
        int16_t ki;
        int16_t kd;
        int16_t v;
        int16_t w;
    };

    struct snd_message{
        uint8_t id;
        int16_t value;
    };

    extern bool useControl;
    extern bool doTwiddle;
    extern bool noControl;



    void setup(uint8_t robot);
    void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len);
    void receiveConfig(bool *control, bool *twiddle, bool *noControl, double *kp, double *ki, double *kd);
    void receiveDataGame(double *v, double *w);
    void receiveDataTwiddle(double *kd, double *ki, double *kp);
    bool isCommunicationLost();
    void sendResponse(double erro);

}

#endif