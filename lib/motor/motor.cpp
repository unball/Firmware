#include "motor.hpp"

namespace Motor {
    
    int8_t motor_direction[2] = {0, 0};

    uint8_t pin1[2] = {AIN1_PIN, BIN1_PIN};
    uint8_t pin2[2] = {AIN2_PIN, BIN2_PIN};
    uint8_t PWM[2] = {PWMA_PIN, PWMB_PIN};

    int8_t getMotorDirection(int8_t motor){
        return motor_direction[motor];
    }

    void setup(void){
        pinMode(PWMA_PIN, OUTPUT);
        pinMode(AIN1_PIN, OUTPUT);
        pinMode(AIN2_PIN, OUTPUT);

        pinMode(STBY_PIN, OUTPUT);

        pinMode(PWMB_PIN, OUTPUT);
        pinMode(BIN1_PIN, OUTPUT);
        pinMode(BIN2_PIN, OUTPUT);

        analogWriteResolution(10);
    }

    void move(uint8_t motor, int32_t power) {
        //Allows current for the motors
        digitalWrite(STBY_PIN, HIGH);

        // Clamp power value to valid PWM range [-1023, 1023]
        power = constrain(power, -1023, 1023);

        //Defines motor direction
        motor_direction[motor] = (power > 0 ? FORWARD : BACKWARD);
        int inPin1;
        int inPin2;
        if (motor_direction[motor] == BACKWARD) {
            inPin1 = HIGH;
            inPin2 = LOW;
        }
        else {
            inPin1 = LOW;
            inPin2 = HIGH;
        }

        //Writes the motor direction and power
        digitalWrite(pin1[motor], inPin1);
        digitalWrite(pin2[motor], inPin2);
        analogWrite(PWM[motor], abs(power));
    }

    //Stops current for the motors
    void stop() {
        Motor::move(MOTOR_RIGHT, 0);
        Motor::move(MOTOR_LEFT, 0);
        digitalWrite(STBY_PIN, LOW);
    }
}