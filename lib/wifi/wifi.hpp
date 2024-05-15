#ifndef WIFI_H
#define WIFI_H
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "../../include/config.h"

namespace Wifi{

    struct rcv_message{
        int8_t id;
        int16_t v;
        int16_t w;
    };
  
    void setup(uint8_t robot);
    void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len);
    void receiveData(double *v, double *w);
    bool isCommunicationLost();
    void sendResponse(double erro);

}

#endif