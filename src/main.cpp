#include <Arduino.h>

#define WEMOS_DEBUG false
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "motor.hpp"
#include "waves.hpp"


Radio::dataStruct vel;

void setup() {	

	//Serial.begin(9600);
	//while(!Serial);

	Radio::setup(0, 3);
	Motor::setup();
}

void loop() {
	#if WEMOS_DEBUG
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
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
		static double v = 0;
		static double w = 0;

		// Lê velocidade do rádio
		Radio::receiveData(&v, &w);

		// Rádio foi perdido, mais de 2s sem mensagens
		if(Radio::isRadioLost()){
			w = 0;
			v = Waves::sine_wave();

			Motor::move(0, v);
			Motor::move(1, v);
		}

		// Executa o controle normalmente com as velocidades de referência
		else {
			// Passa para a planta a saída do controle digital e da malha acoplada
			Motor::move(0, v);
			Motor::move(1, w);
		}

	#endif


}