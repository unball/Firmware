#ifndef MOTOR_H
#define MOTOR_H
#include <Pins/Pins.h>
#define DEBUG_MOTOR false


namespace Motor {
    int8_t motor_direction[2] = {0, 0};

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
        uint8_t pin1[2] = {Pins::AIN1, Pins::BIN1};
        uint8_t pin2[2] = {Pins::AIN2, Pins::BIN2};
        uint8_t PWM[2] = {Pins::PWMA, Pins::PWMB};

        if(power > 255) {
            power = 255;
        }
        if(power < -255){
            power = -255;
        }

        motor_direction[motor] = (power > 0 ? 1:-1);
        int inPin1;
        int inPin2;
        if (motor_direction[motor] == -1) {
            inPin1 = HIGH;
            inPin2 = LOW;
        }
        else {
            inPin1 = LOW;
            inPin2 = HIGH;
        }

        digitalWrite(pin1[motor], inPin1);
        digitalWrite(pin2[motor], inPin2);
        analogWrite(PWM[motor], abs(power));
    }

    void stop(int motorNumber) {
        //digitalWrite(Pins::STBY, LOW);
        move(motorNumber, 0);
    }
}

#endif
