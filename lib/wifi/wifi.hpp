#ifndef WIFI_H
#define WIFI_H
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "../../include/config.h"


namespace Wifi{

    #if PID_TUNNER
    struct snd_message
    {
        int16_t err;
    };

     struct rcv_message
    {
        int16_t id;
        int16_t kp;
        int16_t ki;
        int16_t kd;
        int16_t w;
        int16_t v;
    };
    void receiveData(float* kp, float* ki, float* kd, float* v, float* w);
    void sendWifi(float erro);
    

    #else
    struct rcv_message{
        int16_t id;
        int16_t vl;
        int16_t vr;
    };
    void receiveData(int16_t *vl, int16_t *vr);

    #endif
    void setup_debug(uint8_t robot);
    void setup(uint8_t robot);
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
    bool isCommunicationLost();

}
#endif