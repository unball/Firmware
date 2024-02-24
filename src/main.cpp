#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>

double erro = 0;

void setup() {
	Serial.begin(115200);	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif

	Wifi::setup(ROBOT_NUMBER);
	IMU::setup();
	Motor::setup();
}

void loop() {

	//Wifi::receiveConfig(&useControl, &doTwiddle, &noControl, &kp, &ki, &kd);
	
	#if WEMOS_DEBUG
		static double v; 
		static double w;
		double kp;
		double ki;
		double kd; 
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
		Serial.print("useControl: ");Serial.print(Wifi::useControl);
        Wifi::receiveData(&kp, &ki, &kd, &v, &w);
		Serial.print("v: ");Serial.print(v);Serial.print("\tw: ");Serial.println(w);
		Serial.println("###################");
		//=========End Wifi===========

		//=========Motor===============
		Motor::move(0, 100);
		Motor::move(1, 100);
		//=========End Motor===========
		delay(500);
	#else
		if(true){
			static int32_t previous_t;
			static int32_t t;
			t = micros();

			// Loop de controle deve ser executado em intervalos comportados
			if(t-previous_t >= controlLoopInterval){
				previous_t = t;
				Control::stand();
			}
		}
		if(false){
			Wifi::sendResponse(erro);
		}



	#endif

}