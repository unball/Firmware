#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>
#include "../../include/config.h"

void setup() {
	Wifi::setup(ROBOT_NUMBER);
	Serial.begin(115200);	
	#if WEMOS_DEBUG
  		
		while(!Serial);
		delay(1000);
		Serial.println("START");
		IMU::setup_debug();
		//Wifi::setup_debug(ROBOT_NUMBER);
	#else
		// IMU::setup();
		
	#endif

	Motor::setup();

}

void loop() {
	Serial.println("LOOP!");
	#if WEMOS_DEBUG
		float kp;
		float ki; 
		float kd; 
		float w; 
		float v;  
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
        Wifi::receiveData(&kp, &ki, &kd,  &w, &v);
		Serial.print("kp: ");Serial.print(kp);
		Serial.print("\tki: ");Serial.println(ki);
		Serial.print("\tkd: ");Serial.println(kd);
		Serial.print("\tw: ");Serial.println(w);
		Serial.print("\tv: ");Serial.println(v);
		Serial.println("###################");
		//=========End Wifi===========

		//=========Motor===============
		Motor::move(0, 00);
		Motor::move(1, 00);
		//=========End Motor===========
		delay(500);
	#else
		static int32_t previous_t;
		static int32_t t;
		t = micros();
		#if PID_TUNNER
		// Loop de controle deve ser executado em intervalos comportados
		if(t-previous_t >= controlLoopInterval){
			Serial.println("c0ntr0 interval false 0k!");
			previous_t = t;
			Control::stand(false);
			//Control::actuateNoControl();

		}
		else{
			Serial.println("c0ntr0 interval true 0k!");
			Control::stand(true);
		}
		#else
		if(t-previous_t >= controlLoopInterval){
			previous_t = t;
			Control::actuateNoControl();

		}
		#endif
	

	#endif

}