#include "wifi.hpp"

namespace Wifi{

    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    rcv_message temp_msg;
    rcv_message msg;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    void setup(uint8_t robot){
        robotNumber = robot;

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        if (esp_now_init() != ESP_OK) {
            #if WEMOS_DEBUG
            Serial.println("Erro ao inicializar o ESP-NOW");
            #endif
            return;
        }
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));            
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
        tokenize(incomingData);
	    lastReceived = micros();
        
        //verifica o checksum
        if(temp_msg.checksum == temp_msg.v + temp_msg.w){
            msg = temp_msg;
        }
        else{
            #if WEMOS_DEBUG
                Serial.println("###################");
		        Serial.println("###################");
                Serial.println("ERRO DE CHECKSUM");
                Serial.println("###################");
                Serial.println("###################");
            #endif
        }
        // TODO: lastReceived deveria estar aqui ou em receiveData?
    }
    void tokenize(const uint8_t *data){ //função para tokenizar a string que recebemos do transmissor 
        //modelo esperado de data=="<id,v,w,checksum>"
        if(data==NULL){
            return;
        }
        char* str=new char[strlen((char*)data)+1]; 
        if(str==NULL||str[0]!='<'){
            return;
        }
        strcpy(str,(char*)data);
        str[strlen((char*)data)]='\0';
        str++;
        
        char* token;
        token=strtok(str,","); //trocamos todas as ocorrências de "," por "\0"
        temp_msg.id=std::strtol(token,NULL,10);

        token=strtok(NULL,",");
        temp_msg.v=std::strtol(token,NULL,10);

        token=strtok(NULL,",");
        temp_msg.w=std::strtol(token,NULL,10);

        token=strtok(NULL,",");
        temp_msg.checksum=std::strtol(token,NULL,10);

        delete[] (str-1); //str-1 para incluir o '<' que ignoramos acima.
    }
    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveData(int16_t *v, int16_t *w){
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities and constants
            *v  = msg.v;
            *w  = msg.w;
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