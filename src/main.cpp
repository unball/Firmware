#include <Arduino.h>

#define TEENSY_DEBUG true
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "motor.hpp"
#include "control.hpp"


Radio::dataStruct vel;

void setup() {	

	//Serial.begin(9600);
	//while(!Serial);

	Radio::setup(0, 3);
	Motor::setup();
}

void loop() {
	#if TEENSY_DEBUG
		Serial.println("LOOP!");
		
		//=========Radio==========
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static double v = 0;
        static double w = 0;
		Radio::receiveData(&v, &w);
		Serial.println("Radio:");
		Serial.print("a: ");Serial.print(v);Serial.print("\tb: ");Serial.println(w);
		//=========End Radio===========

		//=========Motor===============
		Motor::move(0, 50);
		Motor::move(1, 50);
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