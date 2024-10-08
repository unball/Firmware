#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>
#include <encoder.hpp>

double erro = 0;
float VV = 0.0;

void setup() {

	Serial.begin(115200);	
	#if WEMOS_DEBUG
  		Serial.begin(115200);
		while(!Serial);
		delay(1000);
		Serial.println("START");
	#endif
	
	Wire.begin(SDA_PIN, SCL_PIN);
	
	Wifi::setup(ROBOT_NUMBER);
	IMU::setup();
	Motor::setup();
	Encoder::setup();
	
	//luz para indicar ligado
	pinMode(15, OUTPUT);
	digitalWrite(15, HIGH);
}

void loop() {
	#if WEMOS_DEBUG
		int16_t v; 
		int16_t w; 
		//Serial.println("LOOP!");
		
		//=========IMU===============
		/*Serial.println("###################");
		Serial.println("IMU:");
		Serial.print("theta: ");Serial.println(IMU::get_w());
		//Serial.println(Encoder::encoder());
		Serial.println("###################");*/

		//=========End IMU===========

		//=========Wifi===============
		/*Serial.println("###################");
		Serial.println("Wi-Fi:");
		Wifi::receiveData(&v, &w);
		Serial.print("v: ");Serial.print(v);Serial.print("w: ");Serial.println(w);
		Serial.println("###################");
		Serial.println("###################");
		Serial.println("macAddress");
		Serial.println(WiFi.macAddress());*/
		//=========End Wifi===========

		//=========Motor===============
		Control::speed2motors(VV,0);
		//=========End Motor===========
		
		//=========Encoder==========
		//Serial.println("Encoder:");
		Encoder::vel enc;	
		enc = Encoder::encoder();
		Serial.print(VV);Serial.print(" ");Serial.println(Encoder::calibration_encoder(enc.motorA));
		//Serial.print("Channel B: ");Serial.println(enc.motorB);
		//double vel = Control::linSpeed(enc);
		//Serial.print("Speed: ");Serial.println(vel);
		//=========End Encoder==========

		delay(1000);
		VV += 0.01;
	#elif CONTROL_TESTER
		Control::test();
	#elif TWIDDLE
		Control::twiddle();
	#elif DEAD_ZONE_TESTER
		Control::deadzone_tester();
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