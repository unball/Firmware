#include <Arduino.h>

#define WEMOS_DEBUG false
#define ROBOT_NUMBER 1

#include "wifi.hpp"
#include "motor.hpp"
#include "waves.hpp"
#include "imu.hpp"

int16_t v = 0;
int16_t w = 0;

void setup() {	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif
	
	#if WEMOS_DEBUG
		IMU::setup_debug();
	#else
		IMU::setup();
	#endif

	Wifi::setup(ROBOT_NUMBER);
	Motor::setup();
	
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
		if(Wifi::isCommunicationLost()){
			w = 0;
			v = Waves::sine_wave();

			Motor::move(0, v);
			Motor::move(1, v);
		}

		else {
			Wifi::receiveData(&v, &w);
			Motor::move(0, v);
			Motor::move(1, w);
		}

	#endif

}