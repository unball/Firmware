#include "wifi.hpp"


//TODO: Resetar ESP 

#define MAX_POWER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values

/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
        lastReceived = micros();
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
    {
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
        // TODO: last Received deveria estar aqui ou em receiveData?
    }

void sendWifi(float ew){
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress, (uint8_t *) &ew, sizeof(ew));
    delay(3);
  
}

    #if PID_TUNNER

    /// @brief Receive data copying from temp struct to global struct
    /// @param kp Constoante proporcional
    /// @param ki Constante integral
    /// @param kd Constante derivativa
    void receiveData(float* kp, float* ki, float* kd, float* w, float* v){
            // Protecting original data
            msg = temp_msg;
            if(msg.id == robotNumber){
                // Demultiplexing and decoding the velocities
                *kp = (float)msg.kp/100;
                *ki = (float)msg.ki/100;
                *kd = (float)msg.kd/100;
                *w = (float)msg.w/100;
                *v = (float)msg.v/100;
            }
        }

    #else
    /// @brief Receive data copying from temp struct to global struct
    /// @param vl reference to the velocity
    /// @param vr reference to the velocity
    void receiveData(int16_t *vl, int16_t *vr){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities
            *vl = msg.vl;
            *vr = msg.vr;
        }
    }
    #endif

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