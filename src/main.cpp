#include <Arduino.h>

#define WEMOS_DEBUG false
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "motor.hpp"
#include "waves.hpp"
#include "battery.hpp"


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
		
		//=========Radio===============
		// Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static double v = 0;
        static double w = 0;
		Radio::receiveData(&v, &w);
		Serial.println("Radio:");
		Serial.print("a: ");Serial.print(v);Serial.print("\tb: ");Serial.println(w);
		//=========End Radio===========

		//=========Bateria===============
		static float voltage = 0.0;
		static float voltagePerc = 0.0;
		Battery::measure(&voltagePerc);
		voltage = Battery::map_float(voltage, 0, 100, 0, MAX_VOLTAGE);
		Serial.println("Bateria:");
		Serial.print("Tensão aproximada: ");Serial.print(voltage);Serial.print("%\tPorcentagem: ");Serial.println(voltagePerc);
		//=========End Bateria===========

		//=========Motor===============
		Motor::move(0, 50);
		Motor::move(1, 50);
		//=========End Motor===========

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

		// Manda velocidades pro motor
		else {
			Motor::move(0, v);
			Motor::move(1, w);
		}

	#endif


}