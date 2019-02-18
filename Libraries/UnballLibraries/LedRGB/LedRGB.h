#ifndef LED_RGB_H
#define LED_RGB_H

#include <Pins/Pins.h>


namespace led{

    void Setup(){
        pinMode(Pins::red, OUTPUT);
        pinMode(Pins::green, OUTPUT);
        pinMode(Pins::blue, OUTPUT);
    }

    void red(){
        digitalWrite(Pins::blue, LOW);
        digitalWrite(Pins::green, LOW);
        digitalWrite(Pins::red, HIGH);
    }

    void green(){
        digitalWrite(Pins::red, LOW);
        digitalWrite(Pins::blue, LOW);
        digitalWrite(Pins::green, HIGH);
    }

    void blue(){
        digitalWrite(Pins::red, LOW);
        digitalWrite(Pins::green, LOW);
        digitalWrite(Pins::blue, HIGH);
    }

    void magenta(){
        digitalWrite(Pins::green, LOW);
        digitalWrite(Pins::red, HIGH);
        digitalWrite(Pins::blue, HIGH);
    }

    void yellow(){
        digitalWrite(Pins::blue, LOW);
        digitalWrite(Pins::green, HIGH);
        digitalWrite(Pins::red, HIGH);
    }

    void cian(){
        digitalWrite(Pins::red, LOW);
        digitalWrite(Pins::blue, HIGH);
        digitalWrite(Pins::green, HIGH);
    }

    void white(){
        digitalWrite(Pins::red, HIGH);
        digitalWrite(Pins::blue, HIGH);
        digitalWrite(Pins::green, HIGH);
    }

    void off(){
        digitalWrite(Pins::red, LOW);
        digitalWrite(Pins::blue, LOW);
        digitalWrite(Pins::green, LOW);
    }

}





#endif
