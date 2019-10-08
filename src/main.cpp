#include <Arduino.h>

#define TEENSY_DEBUG false
#define ROBOT_NUMBER 0

#include "radio.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "control.hpp"


Radio::dataStruct vel;
Imu::imuAll imuData;
double pos(double angVel);
double filter(double var);
void setup() {
	#if (TEENSY_DEBUG || CONTROL_DEBUG || IMU_DEBUG || MOTOR_DEBUG)
	Serial.begin(9600);
	while(!Serial);
	Serial.println("SETUP!");
	#endif
	Radio::setup(0, 3);
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
	static int32_t previous_t;
	static int32_t t;
	t = micros();
	if(t-previous_t >= 1500){
		previous_t = t;
		Control::stand();
		//delay(1);
	}
	#endif
	// Imu::imuAll data = Imu::imuRead();
	// static double bias;
	// static bool calibrated = false;
	// if(!calibrated){
	// 	for(uint16_t i=0; i<5000; i++){
	// 		data = Imu::imuRead();
	// 		bias += data.gyro.z;
	// 	}
	// 	calibrated = true;
	// 	bias = (double)bias/5000.0;
	// }
	// //Serial.print("bias: "); Serial.println(bias);
	// //Serial.print("gyro: "); Serial.println(data.gyro.z);
	// //Serial.print("gyro-bias: "); Serial.println(data.gyro.z-bias);
	// double gyro = filter(data.gyro.z-bias);
	// Serial.print("pos: "); Serial.print(pos(gyro));Serial.print("\r");
	// delay(1);
	//Encoder::vel enc = Encoder::encoder();
	//Radio:: dataStruct message;

}

double pos(double angVel){
	static double pos = 0, oldVel = 0;
	pos = pos + ((angVel + oldVel)/2.0)*(Imu::deltaT()/1000000.0);
	oldVel = angVel;
	return pos;
}

double filter(double var){
	static double varOld;
	varOld = 0.05*var + 0.95*varOld;
	return varOld;
}