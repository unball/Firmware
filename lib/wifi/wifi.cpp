#include "wifi.hpp"

namespace Wifi{

    typedef struct __attribute__((packed)) {
        uint32_t timestamp_us;       // 4 bytes
        float v;                     // 4
        float w;                     // 4
        float u_L;                   // 4
        float u_R;                   // 4
        float omega_L;              // 4
        float omega_R;              // 4
        float w_L;                  // 4
        float w_R;                  // 4
        float theta1_L;             // 4
        float theta2_L;             // 4
        float theta1_R;             // 4
        float theta2_R;             // 4
        float e_L;                  // 4
        float e_R;                  // 4
    } FeedbackPacket;



    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    rcv_message temp_msg;
    rcv_message msg;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    void setup(uint8_t robot) {
        robotNumber = robot;
    
        WiFi.mode(WIFI_STA);
        
        uint8_t newMAC[6] = {0x02, 0x55, 0x4E, 0x42, 0x00, robot};
        esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, newMAC);
    
        if (err == ESP_OK) {
            if (RobotConfig::isDebug()) {
                Serial.print(F("✅ New MAC Address set to: "));
                for (int i = 0; i < 6; i++) {
                    if (i > 0) Serial.print(":");
                    Serial.printf("%02X", newMAC[i]);
                }
                Serial.println();
            }
        } else {
            if (RobotConfig::isDebug()) {
                Serial.print(F("❌ Failed to set MAC Address. Error: "));
                Serial.println(err);
            }
        }
        
        WiFi.disconnect();
    
        if (esp_now_init() != ESP_OK) {
            if (RobotConfig::isDebug()) {
                Serial.println(F("Error initializing ESP-NOW"));
            }
            return;
        }
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        esp_err_t error = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // ou MAC do ESP32 do PC
        peerInfo.channel = 14;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            if (RobotConfig::isDebug()) {
                Serial.println(F("❌ Failed to add peer"));
            }
        }
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
        tokenize(incomingData,len);
	    lastReceived = micros();

        if(temp_msg.checksum == temp_msg.v + temp_msg.w){
            msg = temp_msg;
        }
        else{
            if (RobotConfig::isDebug) {
                Serial.println(F("###################"));
                Serial.println(F("CHECKSUM ERROR"));
                Serial.println(F("###################"));
            }
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
        temp_msg.v=std::strtol(token,NULL,10);

        token=strtok(NULL,",");
        temp_msg.w=std::strtol(token,NULL,10);

        token=strtok(NULL,",");
        temp_msg.checksum=std::strtol(token,NULL,10);
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    /// @return true if the message is from the robot, false otherwise
    bool receiveData(int16_t *v, int16_t *w) {
        if (msg.id == robotNumber) {
            *v = msg.v;
            *w = msg.w;
            return true;
        }
        return false;
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

    void sendFeedback(float v, float w,
                        float u_L, float u_R,
                        float omega_L, float omega_R,
                        float w_L, float w_R,
                        float theta1_L, float theta2_L,
                        float theta1_R, float theta2_R,
                        float e_L, float e_R) {

        FeedbackPacket packet;
        packet.timestamp_us = micros();
        packet.v = v;
        packet.w = w;
        packet.u_L = u_L;
        packet.u_R = u_R;
        packet.omega_L = omega_L;
        packet.omega_R = omega_R;
        packet.w_L = w_L;
        packet.w_R = w_R;
        packet.theta1_L = theta1_L;
        packet.theta2_L = theta2_L;
        packet.theta1_R = theta1_R;
        packet.theta2_R = theta2_R;
        packet.e_L = e_L;
        packet.e_R = e_R;

        esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet));
    }



}