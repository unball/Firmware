#ifndef WIFI_H
#define WIFI_H
#include <WiFi.h>
#include <esp_now.h>
#include "../../include/config.h"

namespace Wifi{

    struct rcv_message{
        int8_t id;
        int16_t v;
        int16_t w;
    };
  
    void setup(uint8_t robot);
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void receiveData(double *v, double *w);
    bool isCommunicationLost();
}

#endif