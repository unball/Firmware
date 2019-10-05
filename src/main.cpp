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


#define DEADZONE 5
int32_t deadzone(int32_t vin){
	return (vin > 0) ? vin+DEADZONE : vin-DEADZONE;
}

void triangular_wave(int32_t *v1, int32_t *v2){
	static int32_t triangular_wave_cont;
	if(triangular_wave_cont >= 1000){
		triangular_incrementer = -1;
	}
	else if(triangular_wave_cont <= -1000){
		triangular_incrementer = 0;
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

void step(int32_t *v1, int32_t *v2){
	static uint32_t step_cont;
	static int8_t step_flag = 0;
	if(step_cont > 200){
		step_flag = 1;
	}
	else {
		step_cont++;
	}
	*v1 = 64*step_flag;
	*v2 = 64*step_flag;
}

void run_straight(int32_t *v1, int32_t *v2){
	*v1 = 64;
	*v2 = 64;
}

void loop() {
	static int32_t previous_t;
	static int32_t t;

	t = micros();

	if(t-previous_t >= 1500){
		Serial.println(t-previous_t);
		int32_t v1,v2;
		step(&v1, &v2);

		previous_t = t;
		Motor::move(0, deadzone(v1));
		Motor::move(1, deadzone(v2));

		Encoder::vel enc = Encoder::encoder();
		Radio::vel message;
		message.vel_A = (int32_t)enc.motorA;
		message.vel_B = (int32_t)enc.motorB;
		message.in_A = v1;
		message.in_B = v2;
		message.time = t;

		Radio::reportMessage(message);
	}
	
	//delay(1);

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