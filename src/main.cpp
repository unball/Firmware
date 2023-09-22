#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define WEMOS_DEBUG false
#define ROBOT_NUMBER 1

#include "motor.hpp"
#include "waves.hpp"
#include "imu.hpp"

typedef struct dataStruct
{
	uint8_t id;
	float v;
	float w;
} dataStruct;

dataStruct temp_vel;

dataStruct vel;

volatile static uint32_t lastReceived;

double v = 0;
double w = 0;

// Callback function, execute when message is received via Wi-Fi
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
{
	memcpy(&temp_vel, incomingData, sizeof(vel));
	lastReceived = micros();
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
	
	#if WEMOS_DEBUG
		IMU::setup_debug();
	#else
		IMU::setup();
	#endif
	Motor::setup();
	lastReceived = micros();
	
}

void loop() {
	#if WEMOS_DEBUG
		Serial.println("LOOP!");

		//=========IMU===============
		Serial.println("###################");
		Serial.println("IMU:");
		Serial.print("theta: ");Serial.println(IMU::get_w());
		Serial.println("###################");
		//=========End IMU===========

		//=========Wifi===============
		Serial.println("###################");
		Serial.println("Radio:");
		Serial.print("v: ");Serial.print(v);Serial.print("\tw: ");Serial.println(w);
		Serial.println("###################");
		//=========End Wifi===========

		//=========Motor===============
		Motor::move(0, 100);
		Motor::move(1, 100);
		//=========End Motor===========
		delay(500);
	#else
		// Communication has been lost
		if((micros() - lastReceived) > RADIO_THRESHOLD){
			// Communication probably failed
			if((micros() - lastReceived) > RADIO_RESET_THRESHOLD)
				ESP.restart();
			w = 0;
			v = Waves::sine_wave();

			Motor::move(0, v);
			Motor::move(1, v);
		}

		else {
			// Protecting original data
			vel = temp_vel;
			if(vel.id == ROBOT_NUMBER){
				v = vel.v;
				w = vel.w;
			}
			Motor::move(0, v);
			Motor::move(1, w);
		}

	#endif

}