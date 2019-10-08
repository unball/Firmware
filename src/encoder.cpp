#include "encoder.hpp"

#define alpha 1

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
        static float prevVA, prevVB;

        double t = timeCounter();
        vel enc;
        presentVelA = (float) (contadorA/t)*Motor::getMotorDirection(0);
        presentVelB = (float) (contadorB/t)*Motor::getMotorDirection(1);

        enc.motorA = alpha* presentVelA + (1-alpha) * prevVA;
        enc.motorB = alpha* presentVelB + (1-alpha) * prevVB;
        
        timeAnt = micros()/1000.0;
        contadorA = 0;
        contadorB = 0;
        prevVA = presentVelA;
        prevVB = presentVelB;
        return enc;
    }
}
