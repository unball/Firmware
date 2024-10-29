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
        tokenize(incomingData,len);
	    lastReceived = micros();

        if(temp_msg.checksum == static_cast<int32_t>(temp_msg.v) + static_cast<int32_t>(temp_msg.w) ){
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
    void tokenize(const uint8_t *data,int len){ //função para tokenizar a string que recebemos do transmissor 
        //modelo esperado de data=="[id,v,w,checksum]"
        if(data==NULL){
            return;
        }
        char str[len+1];
        memcpy(str,data,len);
        if(str[0]!='['){
            return;
        }
        str[len]='\0';

        char* token;
        //não é necessário fazer verificação no strtok pois só temos 4 variáveis e não um número indefinido
        token=strtok(str,","); //trocamos todas as ocorrências de "," por "\0"
        temp_msg.id=token[1]-'0';   //sabendo que a variável ID tem range [0,2] usamos essa forma, se mudar por algum motivo...
                                    //faça igual abaixo, mas cuidado com token[0] que é '<'
                                    //com ponteiro é mais fácil de driblar isso mas não quero usar alocação dinâmica pois a stack é mais rápida =)

        token=strtok(NULL,",");
        temp_msg.v=std::strtof(token,NULL);

        token=strtok(NULL,",");
        temp_msg.w=std::strtof(token,NULL);

        token=strtok(NULL,",");
        temp_msg.checksum=std::strtol(token,NULL,10);
    }
    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveData(float *v, float *w){
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