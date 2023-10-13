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

	Wifi::setup(ROBOT_NUMBER);
	IMU::setup();				// Must be after setting up the Wifi, in case the MPU is not working properly
	Motor::setup();
}

void loop() {
	#if WEMOS_DEBUG
		static double v; 
		static double w; 
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
        Wifi::receiveData(&v, &w);
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

		// Control loop must be executed in steady intervals
		if(t-previous_t >= controlLoopInterval){
			previous_t = t;
			Control::stand();
		}

	#endif

}