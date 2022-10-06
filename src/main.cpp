#include <Arduino.h>

#define WEMOS_DEBUG true
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "motor.hpp"
#include "waves.hpp"
#include "battery.hpp"


Radio::dataStruct vel;

void setup() {	
	#if WEMOS_DEBUG
		Serial.begin(9600);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif
	Radio::setup(ROBOT_NUMBER, 3);
	Motor::setup();
	pinMode(BATT,INPUT);
	
}

void loop() {
	#if WEMOS_DEBUG
		Serial.println("LOOP!");
		
		//=========Radio===============
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static double vl;
        static double vr;
		Radio::receiveData(&vl, &vr);
		Serial.println("###################");
		Serial.println("Radio:");
		Serial.print("vl: ");Serial.print(vl);Serial.print("\tvr: ");Serial.println(vr);
		Serial.println("###################");
		//=========End Radio===========

		//=========Bateria===============
		static double voltage;
		Battery::measure(&voltage);
		Serial.println("Bateria:");
		Serial.print("Tensão aproximada: ");Serial.println(voltage);
		//=========End Bateria===========

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