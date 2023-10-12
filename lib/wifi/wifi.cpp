#include "wifi.hpp"

//TODO: Resetar ESP 

namespace Wifi{

    rcv_message temp_msg;

    rcv_message msg;

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
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
        // TODO: last Received deveria estar aqui ou em receiveData?
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    /// @return True if the control will be used 
    bool receiveData(double *v, double *w){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities
            *v = msg.v * 2.0 / 32767;
            *w = msg.w * 64.0 / 32767;
        }
        return bool(msg.control);
    }

    bool isCommunicationLost(){
        if((micros() - lastReceived) > communicationTimeout){
			// Communication probably failed
			if((micros() - lastReceived) > resetTimeout)
				ESP.restart();
            return true;
		}
        return false;
    }

}