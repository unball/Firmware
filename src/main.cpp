#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>
#include <encoder.hpp>

void setup() {

	Serial.begin(115200);	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		Serial.println("START");
		delay(1000);
	#endif
	Encoder::setup();
}

void loop() {

	#if WEMOS_DEBUG
		static int32_t previous_t;
		static int32_t t;
		t = micros();

        static double v = 1.0;

		// Loop de controle deve ser executado em intervalos comportados
		if(t-previous_t >= controlLoopInterval){
			previous_t = t;

			Motor::move(0, v);
			Motor::move(1, v);

			Serial.println("Encoder:");
			Encoder::vel enc;
			enc = Encoder::encoder();
			Serial.print("Channel A: ");Serial.println(enc.motorA);
			Serial.print("Channel B: ");Serial.println(enc.motorB);
			double vel = Control::linSpeedTest(enc);
			Serial.print("Speed: ");Serial.println(vel);
		}

		//=========Motor===============
		// v = Waves::sine_wave();

		//=========End Motor===========
		
		// //=========Encoder==========
		// Serial.println("Encoder:");
		// Encoder::vel enc;
		// enc = Encoder::encoder();
		// Serial.print("Channel A: ");Serial.println(enc.motorA);
		// Serial.print("Channel B: ");Serial.println(enc.motorB);
		// double vel = Control::linSpeedTest(enc);
		// Serial.print("Speed: ");Serial.println(vel);
		// //=========End Encoder==========

		delay(200);
	#elif CONTROL_TESTER
		Control::test();
	#elif TWIDDLE
		Control::twiddle();
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