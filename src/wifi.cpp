#include "wifi.hpp"

//TODO: Restar ESP 

namespace Wifi{

    const int communicationTimeout = 500000;

    dataStruct temp_vel;

    dataStruct vel;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    void setup_debug(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        if (esp_now_init() != 0) {
            Serial.println("Erro ao inicializar o ESP-NOW");
            return;
        }

        esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
        esp_now_register_recv_cb(OnDataRecv);
    }

    void setup(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        if (esp_now_init() != 0) {
            return;
        }
        WiFi.setOutputPower(MAX_POWER);

        esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
        esp_now_register_recv_cb(OnDataRecv);
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
    {
        memcpy(&temp_vel, incomingData, sizeof(vel));
	    lastReceived = micros();
    }

    void receiveData(double *v, double *w){
        // Protecting original data
        vel = temp_vel;
        if(vel.id == robotNumber){
            *v = vel.v;
            *w = vel.w;
        }
    }

    bool isCommunicationLost(){
        if((micros() - lastReceived) > communicationTimeout){
			// Communication probably failed
			// if((micros() - lastReceived) > RADIO_RESET_THRESHOLD)
			// 	ESP.restart();
            return true;
		}
        return false;
    }

}