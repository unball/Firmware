#include <Arduino.h>

#define TEENSY_DEBUG false
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "motor.hpp"
#include "control.hpp"


Radio::dataStruct vel;

void setup() {

	#if (TEENSY_DEBUG || CONTROL_DEBUG || IMU_DEBUG || MOTOR_DEBUG)
		Serial.begin(9600);
		while(!Serial);
		Serial.println("SETUP!");
	#endif
	

	/*Serial.begin(9600);
	while(!Serial);*/

	Radio::setup(0, 3);
	Motor::setup();
}

void loop() {
	#if TEENSY_DEBUG
		Serial.println("LOOP!");
		
		//=========Radio==========
		vel.B = 56;
		vel.A = 47;
		Radio::receiveData(&vel);
		Serial.println("Radio:");
		Serial.print("a: ");Serial.print(vel.A);Serial.print("\tb: ");Serial.println(vel.B);
		//=========End Radio===========

		//=========Motor==========
		Motor::move(0, 100);
		Motor::move(1, 100);
		//=========End Motor==========

		delay(100);
	#else
		static int32_t previous_t;
		static int32_t t;
		t = micros();

		// Loop de controle deve ser executado em intervalos comportados (2ms)
		if(t-previous_t >= 2000){
			previous_t = t;
			Control::stand();
		}
	#endif


}