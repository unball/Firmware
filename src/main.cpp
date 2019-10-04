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
	Serial.begin(9600);
	while(!Serial);
	#if (TEENSY_DEBUG || CONTROL_DEBUG || IMU_DEBUG || MOTOR_DEBUG)
	Serial.println("SETUP!");
	#endif
	Radio::setup(0, 3);
	Imu::Setup();
	Motor::setup();
	Encoder::setup();
}

int8_t wave_flag = 1;
int8_t triangular_incrementer = 1;

void triangular_wave(int32_t *v1, int32_t *v2){
	static int32_t triangular_wave_cont;
	if(triangular_wave_cont >= 1000){
		triangular_incrementer = -10;
	}
	else if(triangular_wave_cont <= -1000){
		triangular_incrementer = 10;
	}
	triangular_wave_cont += triangular_incrementer;
	*v1 = triangular_wave_cont*64/1000;
	*v2 = triangular_wave_cont*64/1000;
}

void square_wave(int32_t *v1, int32_t *v2){
	static uint32_t square_wave_cont;
	if(square_wave_cont > 200){
		wave_flag = -1*wave_flag;
		square_wave_cont = 0;
	}
	*v1 = 64*wave_flag;
	*v2 = 64*wave_flag;
	square_wave_cont++;
}

void run_straight(int32_t *v1, int32_t *v2){
	*v1 = 64;
	*v2 = 64;
}

void loop() {
	int32_t v1,v2;
	triangular_wave(&v1, &v2);

	Motor::move(0, v1);
	Motor::move(1, v2);

	Encoder::vel enc = Encoder::encoder();
	Radio::vel message;
	message.vel_A = (int32_t)enc.motorA;
	message.vel_B = (int32_t)enc.motorB;
	message.in_A = v1;
	message.in_B = v2;
	message.time = micros();

	Radio::reportMessage(message);

	delay(1);

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
	//Control::stand();
	//delay(1);
	#endif
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