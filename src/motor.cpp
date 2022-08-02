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

        pinMode(PWMB_PIN, OUTPUT);
        pinMode(BIN1_PIN, OUTPUT);
        pinMode(BIN2_PIN, OUTPUT);
    }

    void move(uint8_t motor, int32_t power) {
        //Allows current for the motors

        //Saturador
        power = power > 255 ? 255 : power;
        power = power < -255 ? -255 : power;

        //Defines motor direction
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

        //Writes the motor direction and power
        digitalWrite(pin1[motor], inPin1);
        digitalWrite(pin2[motor], inPin2);
        analogWrite(PWM[motor], abs(power));
        Serial.print("Motor: ");Serial.print(motor);Serial.print("%\tinpin1 ");Serial.print(inPin1);
        Serial.print("\tinpin2: ");Serial.println(inPin2);
    }

    //Stops current for the motors
    void stop() {
        digitalWrite(pin1[0], LOW);
        digitalWrite(pin1[1], LOW);
        digitalWrite(PWM[0], HIGH);

        digitalWrite(pin2[0], LOW);
        digitalWrite(pin2[1], LOW);
        digitalWrite(PWM[1], HIGH);

    }
}