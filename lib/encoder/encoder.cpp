#include "encoder.hpp"

#define alpha 0.95
#define beta 0.05

const int watchdog_timer = 500000; // us

namespace Encoder {

    hw_timer_t *Timer0_Cfg = NULL;
    hw_timer_t *Timer1_Cfg = NULL;

    volatile double present_speed_A;
    volatile double present_speed_B;

    void IRAM_ATTR Ext_INT1_ISR() {
        detachInterrupt(ENC_MOTOR_A_CHA_PIN);
        
        uint64_t T_us = timerReadMicros(Timer0_Cfg);

        int direction = digitalRead(ENC_MOTOR_A_CHB_PIN);
        direction = direction == 1? 1:-1;

        present_speed_A = (double) (direction)*(2*PI)/(12*(T_us*1e-6));

        timerRestart(Timer0_Cfg);

        attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
    }

    void IRAM_ATTR Ext_INT2_ISR() {
        detachInterrupt(ENC_MOTOR_B_CHA_PIN);
        
        uint64_t T_us = timerReadMicros(Timer1_Cfg);

        int direction = digitalRead(ENC_MOTOR_B_CHB_PIN);
        direction = direction == 1? 1:-1;

        present_speed_B = (double) (direction)*(2*PI)/(12*(T_us*1e-6));

        timerRestart(Timer1_Cfg);

        attachInterrupt(ENC_MOTOR_B_CHA_PIN, Ext_INT2_ISR, RISING);
    }

    void setup() {
        // Motor A
        pinMode(ENC_MOTOR_A_CHA_PIN, INPUT);
        pinMode(ENC_MOTOR_A_CHB_PIN, INPUT);
        attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
        // Configure Timer0
        Timer0_Cfg = timerBegin(0, 2, true);
        timerWrite(Timer0_Cfg, 0);
        timerStart(Timer0_Cfg);

        // Motor B
        pinMode(ENC_MOTOR_B_CHA_PIN, INPUT);
        pinMode(ENC_MOTOR_B_CHB_PIN, INPUT);
        attachInterrupt(ENC_MOTOR_B_CHA_PIN, Ext_INT2_ISR, RISING);
        // Configure Timer0
        Timer1_Cfg = timerBegin(1, 2, true);
        timerWrite(Timer1_Cfg, 0);
        timerStart(Timer1_Cfg);
    }

    vel getMotorSpeeds() {
        vel result;

        if (timerReadMicros(Timer0_Cfg) > watchdog_timer) {
            present_speed_A = 0;
        }

        if (timerReadMicros(Timer1_Cfg) > watchdog_timer) {
            present_speed_B = 0;
        }

        result.motorLeft = present_speed_A;
        result.motorRight = present_speed_B;

        return result;
    }
}