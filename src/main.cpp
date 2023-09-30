#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define WEMOS_DEBUG false
#define ROBOT_NUMBER 1
#define WIFI_TIMEOUT 500000
#define WIFI_RESET_TIMEOUT 30000000

#include "motor.hpp"
#include "waves.hpp"


typedef struct dataStruct
{
	int16_t id;
	int16_t vl;
	int16_t vr;
} dataStruct;

//Cria uma struct_message chamada myData
dataStruct vel;

volatile static uint32_t lastReceived;

double vl = 0;
double vr = 0;

//Funcao de Callback executada quando a mensagem for recebida
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
{
	memcpy(&vel, incomingData, sizeof(vel));
	if(vel.id == ROBOT_NUMBER){
		vl = vel.vl;
		vr = vel.vr;
		lastReceived = micros();
	}
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
	
	Motor::setup();
	lastReceived = micros();
	
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
		Serial.print("vl: ");Serial.print(vl);Serial.print("\tvr: ");Serial.println(vr);
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
		// Rádio foi perdido, mais de 2s sem mensagens
		if((micros() - lastReceived) > WIFI_TIMEOUT){
			// Rádio foi disconectado, mais de 5s sem mensagens
			if((micros() - lastReceived) > WIFI_RESET_TIMEOUT)
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