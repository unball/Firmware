#include "encoder.hpp"

#define alpha 1
#define beta 0.05

namespace Encoder {

    volatile uint64_t counter_A = 0;
    volatile uint64_t counter_B = 0;
    uint32_t prev_time = 0;
    double present_vel;
    volatile uint64_t counter = 0;
    double present_speed;
    
    void add_A(){
        counter_A++;
    }
    void add_B(){
        counter_B++;
    }

    void setup() {
        attachInterrupt(digitalPinToInterrupt(CHANNEL_A_PIN), add_A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(CHANNEL_B_PIN), add_B, CHANGE);
        // attachInterrupt(CHANNEL_A_PIN, calc, CHANGE);
        // attachInterrupt(CHANNEL_B_PIN, calc, CHANGE);
    }

    void calc() {
        uint32_t static curr_time, prev_time; // tem que ver como iniciar

        curr_time = micros();
        uint32_t T_us = curr_time - prev_time; // importante estar em uint32, pois a diferença é sempre igual ao tempo
        present_speed = (2*PI)/(2*48*(T_us*1e-6));
        prev_time = curr_time;
    }

    void resetEncoders() {
        counter_A = 0;
        counter_B = 0;
    }

    uint32_t time_counter() {
        uint32_t deltaT = micros() - prev_time;
        return deltaT;
    }


    double get_ticks_per_us() {
        uint32_t t = time_counter();

        detachInterrupt(digitalPinToInterrupt(CHANNEL_A_PIN));
        detachInterrupt(digitalPinToInterrupt(CHANNEL_B_PIN));

        present_vel = (double) ((1.0*(counter_A+counter_B))/t);

        attachInterrupt(digitalPinToInterrupt(CHANNEL_A_PIN), add_A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(CHANNEL_B_PIN), add_B, CHANGE);
        
        prev_time = micros();
        resetEncoders();
        return present_vel;
    }
}

    // vel encoder() {
    //     static float prevVA, prevVB;

    //     double t = time_counter();
    //     vel enc;

    //     detachInterrupt(CHANNEL_A_PIN);
    //     detachInterrupt(CHANNEL_B_PIN);

    //     presentVelA = (double) ((1.0*counter_A)/t)*Motor::getMotorDirection(0);
    //     presentVelB = (double) ((1.0*counter_B)/t)*Motor::getMotorDirection(1);

    //     attachInterrupt(CHANNEL_A_PIN, add_A, RISING);
    //     attachInterrupt(CHANNEL_B_PIN, add_B, RISING);

    //     enc.motorA = alpha* presentVelA + (1-alpha) * prevVA;
    //     enc.motorB = alpha* presentVelB + (1-alpha) * prevVB;
        
    //     prev_time = micros()/1000.0;
    //     counter_A = 0;
    //     counter_B = 0;
    //     prevVA = presentVelA;
    //     prevVB = presentVelB;
    //     return enc;
    // }