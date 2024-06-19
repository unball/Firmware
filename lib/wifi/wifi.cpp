#include "wifi.hpp"

namespace Wifi{

    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    rcv_message temp_msg;
    rcv_message msg;

    // const byte numChars = 32;
    char receivedChars;   

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    void setup(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        if (esp_now_init() != 0) {
            #if WEMOS_DEBUG
            Serial.println("Erro ao inicializar o ESP-NOW");
            #endif
            return;
        }
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        esp_now_register_recv_cb(OnDataRecv);            
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
        memcpy(&receivedChars, incomingData, sizeof(receivedChars));
	    lastReceived = micros();
        
        //verifica o checksum
        // if(temp_msg.checksum == temp_msg.v + temp_msg.w){
        //     msg = temp_msg;
        // }
        // TODO: lastReceived deveria estar aqui ou em receiveData?
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveData(char* command){
        // if(msg.id == robotNumber){
        //     // Demultiplexing and decoding the velocities and constants
        //     *v  = msg.v;
        //     *w  = msg.w;
        // }
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");


        Serial.println("DOHOFAHDIHFODFDHHOHFDOHFA");
        Serial.println(receivedChars);
        Serial.println("DOHOFAHDIHFODFDHHOHFDOHFA");
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");
        Serial.println("\n");
        
        *command = receivedChars;
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