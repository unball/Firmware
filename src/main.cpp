#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>
#include <encoder.hpp>

// Definir os parâmetros do PWM
const int pwmPin = 15; // Pino D15
const int pwmChannel = 0;
const int pwmFreq = 10000; // Frequência em Hz
const int pwmResolution = 10; // Resolução do PWM (8 bits)

void setup() {

	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		Serial.println("START");
		delay(1000);
	#endif
	Encoder::setup();

	// Configurar o canal PWM
	ledcSetup(pwmChannel, pwmFreq, pwmResolution);
	
	// Associar o canal PWM ao pino
	ledcAttachPin(pwmPin, pwmChannel);

	int dutyCycle = 1023;
	ledcWrite(pwmChannel, dutyCycle);

	delay(500);
	dutyCycle = 500;
	ledcWrite(pwmChannel, dutyCycle);


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
			double ticks;

			// Encoder::vel enc;
			ticks = Encoder::get_w();
			Serial.println(ticks);

			// Motor::move(0, v);
			// Motor::move(1, v);

			// Serial.println("Encoder:");
			// Encoder::vel enc;
			// enc = Encoder::encoder();
			// Serial.print("Channel A: ");Serial.println(enc.motorA);
			// Serial.print("Channel B: ");Serial.println(enc.motorB);
			// double vel = Control::linSpeedTest(enc);
			// Serial.print("Speed: ");Serial.println(vel);


			//// Velocidades para plotar
			// Encoder::vel enc;
			// enc = Encoder::encoder();
			// // Serial.print(enc.motorA);Serial.print(",");
			// // Serial.println(enc.motorB);
			// // Serial.print(",");
			// double vel = Control::linSpeedTest(enc);
			// Serial.println((float)vel);
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

		delay(50);
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