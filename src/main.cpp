#include <Arduino.h>

#define TEENSY_DEBUG false

#include "radio.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "control.hpp"


Radio::dataStruct vel;
Imu::imuAll imuData;
void setup() {
	#if TEENSY_DEBUG
	Serial.begin(9600);
	while(!Serial);
	Serial.println("SETUP!");
	#endif
	Radio::setup(1, 3);
	Imu::Setup();
	Motor::setup();
	Encoder::setup();
}

void loop() {
	#if TEENSY_DEBUG
	Serial.println("LOOP!");
	
	//=========Radio==========
	vel.B = 56;
	vel.A = 47;
	Radio::receiveData(&vel);
	Serial.println("Radio:");
	Serial.print("a: ");Serial.print(vel.A);Serial.print("\tb: ");Serial.println(vel.B);
	//=========End Radio===========

	//=========IMU==========  
	Serial.println("IMU:");
	imuData = Imu::imuRead();
	Serial.println("accel:");
	Serial.print("x: ");Serial.println(imuData.accel.x);
	Serial.print("y: ");Serial.println(imuData.accel.y);
	Serial.print("z: ");Serial.println(imuData.accel.z);
	Serial.println("gyro:");
	Serial.print("x: ");Serial.println(imuData.gyro.x);
	Serial.print("y: ");Serial.println(imuData.gyro.y);
	Serial.print("z: ");Serial.println(imuData.gyro.z);
	//=========End IMU==========

	//=========Motor==========
	Motor::move(0, 100);
	Motor::move(1, 100);
	//=========End Motor==========

	//=========Encoder==========
	Serial.println("Encoder:");
	Encoder::vel enc;
	enc = Encoder::encoder();
	Serial.print("Channel A: ");Serial.println(enc.motorA);
	Serial.print("Channel B: ");Serial.println(enc.motorB);
	//=========End Encoder==========

	delay(100);
	#else
	Control::stand();
	#endif
}