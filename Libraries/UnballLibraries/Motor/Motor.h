#ifndef MOTOR_H
#define MOTOR_H
#include <Pins/Pins.h>
#define DEBUG_MOTOR false


namespace Motor {
    long motorA_direction=0;
    long motorB_direction=0;

    void Setup(){
        pinMode(Pins::PWMA, OUTPUT);
        pinMode(Pins::AIN1, OUTPUT);
        pinMode(Pins::AIN2, OUTPUT);

        //pinMode(Pins::STBY, OUTPUT);

        pinMode(Pins::PWMB, OUTPUT);
        pinMode(Pins::BIN1, OUTPUT);
        pinMode(Pins::BIN2, OUTPUT);

        //analogWriteFrequency(Pins::PWMA, 35156.25);  //ideal frequency for 10 bits resolution
        //analogWriteFrequency(Pins::PWMB, 35156.25);
        //analogWriteResolution(10); //0 - 1023
    }

    void move(uint8_t motor, int32_t power) {
        //digitalWrite(Pins::STBY, HIGH);
        uint8_t pin1, pin2;
        uint8_t PWM;
        if (motor == 0) {
        pin1 = Pins::AIN1;
        pin2 = Pins::AIN2;
        PWM = Pins::PWMA;
        }
        else {
        pin1 = Pins::BIN1;
        pin2 = Pins::BIN2;
        PWM = Pins::PWMB;
        }

        //power = map(power,-100,100,-255,255);
        if(power > 255) {
            power = 255;
        }
        if(power < -255){
            power = -255;
        }

        int  motor_direction = (power > 0 ? -1:1);
        boolean inPin1 = LOW;
        boolean inPin2 = HIGH;
        if (motor_direction == -1) {
            inPin1 = HIGH;
            inPin2 = LOW;
        }
        else {
            inPin1 = LOW;
            inPin2 = HIGH;
        }

        digitalWrite(pin1, inPin1);
        digitalWrite(pin2, inPin2);
        analogWrite(PWM, abs(power));
    }

    void stop(int motorNumber) {
        //digitalWrite(Pins::STBY, LOW);
        move(motorNumber, 0);
    }
}

#endif
