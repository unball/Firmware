#include "encoder.hpp"

#define alpha 1

namespace Encoder {

    volatile double contadorA = 0;
    volatile double contadorB = 0;
    double timeAnt = 0;
    double presentVelA;
    double presentVelB;
    
    void somaA(){
        contadorA += 0.0122718463;
    }
    void somaB(){
        contadorB += 0.0122718463;
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
        //desabilita as interrupções para não causar conflito na hr de usar as variaveis
        detachInterrupt(CHANNEL_A_PIN);
        detachInterrupt(CHANNEL_B_PIN);
        presentVelA = (float) (contadorA/t)*Motor::getMotorDirection(0);
        presentVelB = (float) (contadorB/t)*Motor::getMotorDirection(1);
        //habilita as interupções novamente
        attachInterrupt(CHANNEL_A_PIN, somaA, RISING);
        attachInterrupt(CHANNEL_B_PIN, somaB, RISING);

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
