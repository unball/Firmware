#include "wifi.hpp"

//TODO: Restar ESP 

namespace Wifi{

    dataStruct temp_vel;

    dataStruct vel;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    int16_t v = 0;
    int16_t w = 0;

    void setup(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        if (esp_now_init() != 0) {
            Serial.println("Erro ao inicializar o ESP-NOW");
            return;
        }

        esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
        esp_now_register_recv_cb(OnDataRecv);
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
    {
        memcpy(&temp_vel, incomingData, sizeof(vel));
	    lastReceived = micros();
    }

    void receiveData(int16_t *v, int16_t *w){
        // Protecting original data
        vel = temp_vel;
        if(vel.id == robotNumber){
            *v = vel.v;
            *w = vel.w;
        }
    }

    bool isCommunicationLost(){
        if((micros() - lastReceived) > RADIO_THRESHOLD){
			// Communication probably failed
			// if((micros() - lastReceived) > RADIO_RESET_THRESHOLD)
			// 	ESP.restart();
            return true;
		}
        return false;
    }

}