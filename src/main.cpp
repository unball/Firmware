#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define WEMOS_DEBUG true
#define ROBOT_NUMBER 1

#include "radio.hpp"
#include "motor.hpp"
#include "waves.hpp"

typedef struct dataStruct
{
	int16_t vl;
	int16_t vr;
} dataStruct;

//Cria uma struct_message chamada myData
dataStruct vel;

//Funcao de Callback executada quando a mensagem for recebida
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&vel, incomingData, sizeof(vel));
  Serial.println();
  Serial.print("Bytes recebidos: ");
  Serial.println(len);
  Serial.print("String: (");
  Serial.print(vel.vl);
  Serial.println(")");
  Serial.print("String: (");
  Serial.print(vel.vr);
  Serial.println(")");
}

void setup() {	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif

	WiFi.mode(WIFI_STA);
	if (esp_now_init() != 0) {
		Serial.println("Erro ao inicializar o ESP-NOW");
		return;
	}
	esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  	esp_now_register_recv_cb(OnDataRecv);
	
	Radio::setup(ROBOT_NUMBER, 3);
	Motor::setup();
	
}

void loop() {
	#if WEMOS_DEBUG
		Serial.println("LOOP!");
		
		//=========Radio===============
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        // static double vl;
        // static double vr;
		// Radio::receiveData(&vl, &vr);
		// Serial.println("###################");
		// Serial.println("Radio:");
		// Serial.print("vl: ");Serial.print(vl);Serial.print("\tvr: ");Serial.println(vr);
		// Serial.println("###################");
		//=========End Radio===========

		//=========Wifi===============
		Serial.println("###################");
		Serial.println("Radio:");
		Serial.print("vl: ");Serial.print(vel.vl);Serial.print("\tvr: ");Serial.println(vel.vr);
		Serial.println("###################");
		//=========End Wifi===========

		//=========Motor===============
		Motor::move(0, 100);
		Motor::move(1, 100);
		//if(Radio::isRadioLost()){
		//	vr = 0;
		//	vl = Waves::sine_wave();

		//	Motor::move(0, vl);
		//	Motor::move(1, vl);
		//}
		//=========End Motor===========
		delay(500);
	#else
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
		static double vl;
		static double vr;

		// Lê velocidade do rádio
		Radio::receiveData(&vl, &vr);

		// Rádio foi perdido, mais de 2s sem mensagens
		if(Radio::isRadioLost()){
			// Rádio foi disconectado, mais de 5s sem mensagens
			if(Radio::isRadioDisconnected())
				ESP.restart();
			vr = 0;
			vl = Waves::sine_wave();

			Motor::move(0, vl);
			Motor::move(1, vl);
		}

		// Manda velocidades pro motor
		else {
			Motor::move(0, vl);
			Motor::move(1, vr);
		}

	#endif

}