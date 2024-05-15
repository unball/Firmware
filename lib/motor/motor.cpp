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
    }

    int16_t deadzone(int16_t vin, int16_t up, int16_t down){
        if(vin != 0){
            return vin+up ? vin > 0 : vin - abs(down);
        }
        return 0;
    }

    int16_t speed2motors(int16_t v, int16_t w){

        int16_t vr = 0;
        int16_t vl = 0;
        float L = 0.075;
        float r = 0.016;

        // Computa a velocidade angular de rotação de cada roda
        vr = (v + (L/2)*w) / r;
        vl = (v - (L/2)*w) / r;

        // Computa a deadzone do motor.
        int16_t vr_sat_min = vr ? vr < 255 : 255;
        int16_t vl_sat_min = vl ? vl < 255: 255;

        int16_t vr_sat_max = vr_sat_min ? vr_sat_min > -255 : -255;
        int16_t vl_sat_max = vl_sat_min ? vl_sat_min > -255 : -255;

        vr = deadzone(vr_sat_max, 32, -32);
        vl = deadzone(vl_sat_max, 32, -32);

        return vr, vl;

    }

    void move(uint8_t motor, int32_t power) {
        //Allows current for the motors
        digitalWrite(STBY_PIN, HIGH);

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
    }

    //Stops current for the motors
    void stop() {
        digitalWrite(STBY_PIN, LOW);
    }
}