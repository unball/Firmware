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
		Wifi::receiveConfig(&Wifi::useControl, &Wifi::doTwiddle, &Control::kp, &Control::ki, &Control::kd);
		Serial.print("useControl: ");Serial.print(Wifi::useControl);Serial.print("doTwiddle: ");Serial.print(Wifi::doTwiddle);Serial.print("kp: ");Serial.print(Control::kp);Serial.print("ki: ");Serial.print(Control::ki);Serial.print("kd: ");Serial.println(Control::kd);
		Serial.println("###################");
		Wifi::receiveDataGame(&v, &w);
		Serial.print("v: ");Serial.print(v);Serial.print("w: ");Serial.println(w);
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

		// Loop de controle deve ser executado em intervalos comportados
		if(t-previous_t >= controlLoopInterval){
			previous_t = t;
			Control::stand();
		}

	#endif

}