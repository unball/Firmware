#ifndef ENCODER_H
#define ENCODER_H
#include <Pins/Pins.h>
#include <Motor/Motor.h>

namespace Encoder {
  volatile long contadorA = 0;
  volatile long contadorB = 0;
  long contadorA_media = 0;
  long contadorB_media = 0;
  unsigned long timeAnt = 0;
  float presentVelA;
  float presentVelB;

  void somaA(){
    contadorA++;
  }
  void somaB(){
    contadorB++;
  }

  void Setup() {
    pinMode(Pins::channelA, INPUT);
    pinMode(Pins::channelB, INPUT);
    attachInterrupt(Pins::channelA, somaA, RISING);
    attachInterrupt(Pins::channelB, somaB, RISING);
  }

  void resetEncoders() {
      contadorA_media=0;
      contadorB_media=0;
  }

  unsigned long timeCounter(){
      unsigned long deltaT = millis() - timeAnt;
      timeAnt = millis();
      return deltaT;
  }

  void encoder() {

    float a = 0.9; // SEMPRE 0 <= a <= 1
    unsigned long t = timeCounter();
    presentVelA = (float) (contadorA/t);
    presentVelB = (float) (contadorB/t);
    contadorA_media = a*contadorA_media + (1 - a)*Motor::motorA_direction*presentVelA;
    contadorB_media = a*contadorB_media + (1 - a)*Motor::motorB_direction*presentVelB;
    contadorA = 0;
    contadorB = 0;
  }
}

#endif
