#ifndef WIFI_H
#define WIFI_H
#include <WiFi.h>
#include <esp_now.h>
#include <cstdlib>
#include <cstring>
#include <esp_wifi.h>

#include <robot_config.hpp>
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
    bool receiveData(int16_t *v, int16_t *w);
    bool isCommunicationLost();
    void tokenize(const uint8_t *data,int len);
    void sendFeedback(float v, float w, float u_L, float u_R, float omega_L, float omega_R, float w_L, float w_R, float theta1_L, float theta2_L, float theta1_R, float theta2_R, float e_L, float e_R);
}

#endif