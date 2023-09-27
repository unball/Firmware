#include <Arduino.h>

#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>

void setup() {	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif
	
	#if WEMOS_DEBUG
		IMU::setup_debug();
		Wifi::setup_debug(ROBOT_NUMBER);
	#else
		IMU::setup();
		Wifi::setup(ROBOT_NUMBER);
	#endif

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
		Serial.println("Wi-Fi:");
		Serial.print("v: ");Serial.print(v);Serial.print("\tw: ");Serial.println(w);
		Serial.println("###################");
		//=========End Wifi===========

		//=========Motor===============
		Motor::move(0, 100);
		Motor::move(1, 100);
		//=========End Motor===========
		delay(500);
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