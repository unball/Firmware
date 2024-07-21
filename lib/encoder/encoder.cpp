#include "encoder.hpp"

#define alpha 1
#define beta 0.05

namespace Encoder {

    volatile uint64_t counter_A = 0;
    volatile uint64_t counter_B = 0;
    double prev_time = 0;
    double present_vel;
    double presentVelA;
    double presentVelB;
    volatile uint32_t timeVarA = 0, lastTimeA = 0;
    volatile uint32_t timeVarB = 0, lastTimeB = 0;
    
    void add_A(){
        counter_A++;
    }
    void add_B(){
        counter_B++;
    }

    void setup() {
        attachInterrupt(CHANNEL_A_PIN, add_A, RISING);
        attachInterrupt(CHANNEL_B_PIN, add_B, RISING);
    }

    void resetEncoders() {
        counter_A = 0;
        counter_B = 0;
    }

    double time_counter(){
        double deltaT = micros()/1000.0 - prev_time;
        return deltaT;
    }

    vel encoder() {
        static float prevVA, prevVB;

        double t = time_counter();
        vel enc;

        detachInterrupt(CHANNEL_A_PIN);
        detachInterrupt(CHANNEL_B_PIN);

        presentVelA = (double) ((1.0*counter_A)/t)*Motor::getMotorDirection(0);
        presentVelB = (double) ((1.0*counter_B)/t)*Motor::getMotorDirection(1);

        attachInterrupt(CHANNEL_A_PIN, add_A, RISING);
        attachInterrupt(CHANNEL_B_PIN, add_B, RISING);

        enc.motorA = alpha* presentVelA + (1-alpha) * prevVA;
        enc.motorB = alpha* presentVelB + (1-alpha) * prevVB;
        
        prev_time = micros()/1000.0;
        counter_A = 0;
        counter_B = 0;
        prevVA = presentVelA;
        prevVB = presentVelB;
        return enc;
    }

    double get_ticks_per_ms() {
        static double prev_vel;
        double encoder_vel;
        double t = time_counter();

        detachInterrupt(CHANNEL_A_PIN);
        detachInterrupt(CHANNEL_B_PIN);

        present_vel = (double) ((1.0*(counter_A+counter_B))/t);

        attachInterrupt(CHANNEL_A_PIN, add_A, CHANGE);
        attachInterrupt(CHANNEL_B_PIN, add_B, CHANGE);

        encoder_vel = alpha * present_vel + (1-alpha) * prev_vel;
        
        prev_time = micros()/1000.0;
        counter_A = 0;
        counter_B = 0;
        prev_vel = present_vel;
        return present_vel;
    }
}
