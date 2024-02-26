#include "wifi.hpp"

namespace Wifi{

    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
        // TODO: lastReceived deveria estar aqui ou em receiveData?
    }

    /// @brief Receive system's configurations in wether to use control or to do the PID Tunner routine, by copying from temp struct to global struct
    /// @param control reference to the control flag
    /// @param twiddle reference to the PID Tunner flag
    void receiveConfig(bool *control, bool *twiddle, double *kp, double *ki, double *kd){
        // Protecting original data
        msg = temp_msg;
        if(msg.id == robotNumber){
            
            if (msg.control == Mode::no_control){
                *control = false;
                *twiddle = false;
            }            
            else if (msg.control == Mode::control){
                if (!Wifi::useControl){
                    *kp = ((float)msg.kp) / 100;
                    *ki = ((float)msg.ki) / 100;
                    *kd = ((float)msg.kd) / 100;
                }
                *control = true;
                *twiddle = false;
            }
            else if (msg.control == Mode::twiddle){
                *control = false;
                *twiddle = true;
            }
        }
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveDataGame(double *v, double *w){
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities and constants
            *v  = ((float)msg.v) * 2.0 / 32767;
            *w  = ((float)msg.w) * 64.0 / 32767;
        }
    }

    /// @brief Receive data copying from temp struct to global struct
    /// @param kp reference to the proportional gain
    /// @param ki reference to the integral gain
    /// @param kd reference to the derivative gain
    /// @param v  reference to the linear velocity
    /// @param w  reference to the angular velocity
    void receiveDataTwiddle(double *kp, double *ki, double *kd){
        if(msg.id == robotNumber){
            // Demultiplexing and decoding the velocities and constants
            *kp = ((float)msg.kp) / 100;
            *ki = ((float)msg.ki) / 100;
            *kd = ((float)msg.kd) / 100;
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

    void sendResponse(double erro){
        snd_message robotStatus;
        robotStatus.id = ROBOT_NUMBER;
        robotStatus.value = (int16_t)(erro * 100);
        esp_now_send(broadcastAddress, (uint8_t *) &robotStatus, sizeof(snd_message));
    }

}