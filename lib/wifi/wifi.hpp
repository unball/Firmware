#ifndef WIFI_H
#define WIFI_H
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "../../include/config.h"

namespace Wifi{

    struct dataRobot
    {
        int16_t vl;
        int16_t vr;
    };
    

    struct rcv_message
    {
        int16_t id;
        dataRobot data;
        int16_t checksum;
    };

    void setup_debug(uint8_t robot);
    void setup(uint8_t robot);
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
    void receiveData(int16_t *vl, int16_t *vr);
    bool isCommunicationLost();

}

#endif