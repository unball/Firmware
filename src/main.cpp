#include <robot_config.hpp>
#include <control.hpp>
#include <imu.hpp>
#include <motor.hpp>
#include <waves.hpp>
#include <wifi.hpp>
#include <encoder.hpp>

void setup() {
	
	Serial.begin(115200);
	if (RobotConfig::isDebug()) {
		while(!Serial);
		delay(1000);
		Serial.println(F("START"));
	}
	
	RobotConfig::setup();
	Wifi::setup(RobotConfig::getRobotNumber());
	IMU::setup();
	Motor::setup();

	// LED to indicate power on
	pinMode(15, OUTPUT);
	digitalWrite(15, HIGH);
}

void loop() {

	if (RobotConfig::isDebug()) {
		int16_t v; 
		int16_t w; 
		Serial.println("LOOP!");

		//=========IMU===============
		Serial.println("###################");
		Serial.println("IMU:");
		Serial.print("theta: "); Serial.println(IMU::get_w());
		Serial.println("###################");
		//=========End IMU===========

		//=========Wi-Fi===============
		Serial.println("###################");
		Serial.println("Wi-Fi:");
		Wifi::receiveData(&v, &w);
		Serial.print("v: "); Serial.print(v); Serial.print(" w: "); Serial.println(w);
		Serial.println("###################");
		Serial.println("###################");
		Serial.println("macAddress:");
		Serial.println(WiFi.macAddress());
		//=========End Wi-Fi===========

		//=========Motor===============
		Motor::move(MOTOR_LEFT, 100);
		Motor::move(MOTOR_RIGHT, 100);
		//=========End Motor===========
		delay(200);
	} else if (RobotConfig::isControlTester()) {
		Control::test();
	} else if (RobotConfig::isTwiddle()) {
		static int32_t t;
		t = millis();
		if (t > twiddledelay) {
			Control::twiddle();
		}
	} else if (RobotConfig::isDeadZoneTester()) {
		Control::deadzone_tester();
	} else {
		static int32_t previous_t;
		static int32_t t;
		t = micros();

		// Control loop must run at well-defined intervals
		if (t - previous_t >= controlLoopInterval) {
			previous_t = t;
			Control::stand();
		}
	}
}
