#ifndef ENCODER_H
#define ENCODER_H
#include <Pins/Pins.h>
#include <Motor/Motor.h>

namespace Encoder {
    volatile int32_t contadorA = 0;
    volatile int32_t contadorB = 0;
    volatile int32_t contador = 0;
    int32_t contadorA_media = 0;
    int32_t contadorB_media = 0;
    //unsigned long timeAnt = 0;
    //float presentVelA;
    //float presentVelB;

    /*void somaA(){
        contadorA++;
    }*/
    void soma(){
        contador++;
    }

    void Setup() {
        pinMode(Pins::channelA, INPUT);
        pinMode(Pins::channelB, INPUT);
        //attachInterrupt(Pins::channelA, somaA, RISING);
        //attachInterrupt(Pins::channelB, somaB, RISING);
    }

    int32_t interruptEncoderPins(int channel) {
        contador = 0;
        attachInterrupt(channel, soma, RISING);
        delay(1);
        detachInterrupt(channel);
        return contador;  
    }

    void resetEncoders() {
        contadorA_media=0;
        contadorB_media=0;
    }

    /*unsigned long timeCounter(){
        unsigned long deltaT = millis() - timeAnt;
        timeAnt = millis();
        return deltaT;
    }*/

    void encoder() {

        int8_t a = 9; // SEMPRE 0 <= a <= 1
        //unsigned long t = timeCounter();
        //presentVelA = (float) (contadorA/t);
        //presentVelB = (float) (contadorB/t);
        contadorA = interruptEncoderPins(Pins::channelA);
        contadorB = interruptEncoderPins(Pins::channelB);
        //contadorA_media = a*contadorA_media + (1 - a)*Motor::motorA_direction*contadorA;
        //contadorB_media = a*contadorB_media + (1 - a)*Motor::motorB_direction*contadorB;
        contadorA_media = a*contadorA_media + (10 - a)*contadorA;
        contadorA_media = contadorA_media/10;
        contadorB_media = a*contadorB_media + (10 - a)*contadorB;
        contadorB_media = contadorB_media/10;
        //contadorA = 0;
        //contadorB = 0;
    }
}

#endif
