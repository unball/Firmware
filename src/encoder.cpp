#include "encoder.hpp"

namespace Encoder {

    volatile int32_t contadorA = 0;
    volatile int32_t contadorB = 0;
    double timeAnt = 0;
    float presentVelA;
    float presentVelB;
    
    void somaA(){
        contadorA++;
    }
    void somaB(){
        contadorB++;
    }

    void setup() {
        attachInterrupt(CHANNEL_A_PIN, somaA, RISING);
        attachInterrupt(CHANNEL_B_PIN, somaB, RISING);
    }

    void resetEncoders() {
        contadorA = 0;
        contadorB = 0;
    }

    double timeCounter(){
        double deltaT = micros()/1000.0 - timeAnt;
        return deltaT;
    }

    vel encoder() {
        //TODO: low-pass filter
        //int8_t a = 5; // SEMPRE 0 <= a <= 10
        double t = timeCounter();
        vel enc;
        Serial.println("+++++++++++++++++++");
        Serial.print(contadorA); Serial.print("\t"); Serial.print(contadorB); Serial.print("\t"); Serial.println(t);
        Serial.println("+++++++++++++++++++");
        presentVelA = (float) (contadorA/t)*Motor::getMotorDirection(0);
        presentVelB = (float) (contadorB/t)*Motor::getMotorDirection(1);

        enc.motorA = presentVelA;
        enc.motorB = presentVelB;
        timeAnt = micros()/1000.0;
        contadorA = 0;
        contadorB = 0;
        return enc;
    }
}
