#include "encoder.hpp"

#define alpha 1
#define beta 0.05

namespace Encoder {

    volatile uint64_t contadorMotorAChanelA = 0;
    volatile uint64_t contadorMotorBChanelA = 0;
    volatile uint64_t contadorMotorAChanelB = 0;
    volatile uint64_t contadorMotorBChanelB = 0;
    double timeAnt = 0;
    double presentVelA;
    double presentVelB;
    volatile uint32_t timeVarA = 0, lastTimeA = 0;
    volatile uint32_t timeVarB = 0, lastTimeB = 0;
    
    void somaMotorAChanelA(){
        contadorMotorAChanelA++;
    }
    void somaMotorBChanelA(){
        contadorMotorBChanelA++;
    }
    void somaMotorAChanelB(){
        contadorMotorAChanelB++;
    }
    void somaMotorBChanelB(){
        contadorMotorBChanelB++;
    }

    void setup() {
        attachInterrupt(CHANNEL_A_MOTOR_A_PIN, somaMotorAChanelA, RISING);
        attachInterrupt(CHANNEL_B_MOTOR_A_PIN, somaMotorBChanelA, RISING);
        attachInterrupt(CHANNEL_A_MOTOR_B_PIN, somaMotorAChanelB, RISING);
        attachInterrupt(CHANNEL_B_MOTOR_B_PIN, somaMotorBChanelB, RISING);
    }

    void resetEncoders() {
        contadorMotorAChanelA = 0;
        contadorMotorBChanelA = 0;
        contadorMotorAChanelB = 0;
        contadorMotorBChanelB = 0;
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
        detachInterrupt(CHANNEL_A_MOTOR_A_PIN);
        detachInterrupt(CHANNEL_B_MOTOR_A_PIN);
        detachInterrupt(CHANNEL_A_MOTOR_B_PIN);
        detachInterrupt(CHANNEL_B_MOTOR_B_PIN);
        presentVelA = (double) ((contadorMotorAChanelA + contadorMotorAChanelB)/(t*2))*Motor::getMotorDirection(0);
        presentVelB = (double) ((contadorMotorBChanelA + contadorMotorBChanelB)/(t*2))*Motor::getMotorDirection(1);
        
        //habilita as interupções novamente
        attachInterrupt(CHANNEL_A_MOTOR_A_PIN, somaMotorAChanelA, RISING);
        attachInterrupt(CHANNEL_B_MOTOR_A_PIN, somaMotorBChanelA, RISING);
        attachInterrupt(CHANNEL_A_MOTOR_B_PIN, somaMotorAChanelB, RISING);
        attachInterrupt(CHANNEL_B_MOTOR_B_PIN, somaMotorBChanelB, RISING);

        enc.motorA = alpha* presentVelA + (1-alpha) * prevVA;
        enc.motorB = alpha* presentVelB + (1-alpha) * prevVB;
        
        timeAnt = micros()/1000.0;
        resetEncoders();
        prevVA = presentVelA;
        prevVB = presentVelB;
        return enc;
    }

    float calibration_encoder(float v){
        return (46.62597898439301 * v - 0.26845859842018827);
    }
}
