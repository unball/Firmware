#include "encoder.hpp"

#define alpha 0.95
#define beta 0.05

const int watchdog_timer = 500000; // us

namespace Encoder {

    // hw_timer_t *Timer0_Cfg = NULL;
    // bool Measurement_InProgress = false;
    // uint64_t Measured_Time = 0;
    // uint64_t Measured_Freq = 0;

    double present_speed;
    uint32_t prev_time = 0;

    // void IRAM_ATTR Ext_INT1_ISR()  {
    //     if(Measurement_InProgress == false)  {
    //         Measurement_InProgress = true;
    //         timerWrite(Timer0_Cfg, 0);
    //         timerStart(Timer0_Cfg);
    //     }
    //     else{
    //         Measured_Time = timerRead(Timer0_Cfg);
    //         timerStop(Timer0_Cfg);
    //         Measurement_InProgress = false;
    //         if(Measured_Time == 0) {
    //             Measured_Freq = 0;
    //         }
    //         else  {
    //             Measured_Freq = (40000000/Measured_Time);
    //         }
    //     }
    // }

    void setup() {
        pinMode(CHANNEL_A_PIN, INPUT);
        pinMode(CHANNEL_B_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(CHANNEL_A_PIN), calc, RISING);
        // attachInterrupt(digitalPinToInterrupt(CHANNEL_B_PIN), calc, RISING);
        // // Configure Timer0
        // Timer0_Cfg = timerBegin(0, 2, true);
    }

    void IRAM_ATTR calc() {
        uint32_t curr_time = micros();
        uint32_t T_us = curr_time - prev_time;

        present_speed = (2*PI)/(12*(T_us*1e-6));

        prev_time = curr_time;
    }

    double get_w() {

        if ((micros() - prev_time) > watchdog_timer) {
            present_speed = 0;
        }

        return present_speed;
    }
}