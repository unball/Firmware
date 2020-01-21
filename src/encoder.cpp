#include "encoder.hpp"

#define alpha 1
#define beta 0.05

namespace Encoder {

    volatile uint64_t contadorA = 0;
    volatile uint64_t contadorB = 0;
    double timeAnt = 0;
    double presentVelA;
    double presentVelB;
    volatile uint32_t timeVarA = 0, lastTimeA = 0;
    volatile uint32_t timeVarB = 0, lastTimeB = 0;
    
    void somaA(){
        contadorA++;
        // timeVarA = beta*(micros()-lastTimeA) + (1-beta)*timeVarA;
        // lastTimeA = micros();
    }
    void somaB(){
        contadorB++;
        // timeVarB = micros()-lastTimeB;
        // lastTimeB = micros();
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

    // vel encoder(){
    //     static float prevVA, prevVB;
    //     double t = timeCounter();
    //     vel enc;
    //     detachInterrupt(CHANNEL_A_PIN);
    //     detachInterrupt(CHANNEL_B_PIN);
    //     presentVelA = (double) ((timeVarA == 0) ? 0 : 1000.0/(timeVarA));
    //     presentVelB = (double) ((1.0*contadorA)/t)*Motor::getMotorDirection(1);
    //     attachInterrupt(CHANNEL_A_PIN, somaA, RISING);
    //     attachInterrupt(CHANNEL_B_PIN, somaB, RISING);

    //     enc.motorA = alpha* presentVelA + (1-alpha) * prevVA;
    //     enc.motorB = alpha* presentVelB + (1-alpha) * prevVB;

    //     timeAnt = micros()/1000.0;
    //     contadorA = 0;
    //     contadorB = 0;
    //     prevVA = enc.motorA;
    //     prevVB = enc.motorB;
    //     return enc;
    // }

    vel encoder() {
        static float prevVA, prevVB;

        double t = timeCounter();
        vel enc;
        //desabilita as interrupções para não causar conflito na hr de usar as variaveis
        detachInterrupt(CHANNEL_A_PIN);
        detachInterrupt(CHANNEL_B_PIN);
        presentVelA = (double) ((1.0*contadorA)/t)*Motor::getMotorDirection(0);
        presentVelB = (double) ((1.0*contadorB)/t)*Motor::getMotorDirection(1);
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
