#ifndef WIFI_H
#define WIFI_H
#include <WiFi.h>
#include <esp_now.h>
#include <cstdlib>
#include <cstring>
#include "../../include/config.h"

namespace Wifi{

    struct rcv_message{
        int8_t id;
        float v;
        float w;
        int32_t checksum;
    };
  
    void setup(uint8_t robot);
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void receiveData(float *v, float *w);
    bool isCommunicationLost();
    void tokenize(const uint8_t *data,int len);
}

#endif