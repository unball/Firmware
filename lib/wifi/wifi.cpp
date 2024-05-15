#include "wifi.hpp"

//TODO: Resetar ESP 

namespace Wifi{

    rcv_message temp_msg;

    rcv_message msg;

    dataRobot robot_message;

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

        /* Faz o checksum */
        int16_t checksum = 0;
        for(int i=0 ; i<3 ; i++){
            checksum += temp_msg.data.v + temp_msg.data.w;
        }

        /* Verifica o checksum */
        if(checksum == temp_msg.checksum){
            /* Copia para o buffer global de robot_message */
            robot_message = temp_msg.data;

            /* Reporta que deu certo */
            Serial.printf("%d\t%d\t%d\n", checksum, robot_message.v, robot_message.w);
        
        }
        else {
            Serial.printf("Checksum errado\n");
        }


	    lastReceived = micros();



        // TODO: last Received deveria estar aqui ou em receiveData?
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param vl reference to the velocity
    /// @param vr reference to the velocity
    void receiveData(int16_t *v, int16_t *w){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities
            *v = msg.data.v;
            *w = msg.data.w;
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