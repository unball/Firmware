#include "wifi.hpp"

namespace Wifi{

    rcv_message temp_msg;

    rcv_message msg;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    bool useControl = false;
    bool doTwiddle = false;

    void setup(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        if (esp_now_init() != 0) {
            #if WEMOS_DEBUG
            Serial.println("Erro ao inicializar o ESP-NOW");
            #endif
            return;
        }
        WiFi.setOutputPower(MAX_POWER);

        esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
        esp_now_register_recv_cb(OnDataRecv);

        receiveConfig(&useControl, &doTwiddle);
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
    {
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
        // TODO: lastReceived deveria estar aqui ou em receiveData?
    }

    /// @brief Receive system's configurations in wether to use control or to do the PID Tunner routine, by copying from temp struct to global struct
    /// @param control reference to the control flag
    /// @param twiddle reference to the PID Tunner flag
    void receiveConfig(bool *control, bool *twiddle){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            *control = msg.control;
            *twiddle = msg.control;
        }
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveData(double *v, double *w){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities
            *v = msg.v * 2.0 / 32767;
            *w = msg.w * 64.0 / 32767;
        }
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