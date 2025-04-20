#include "encoder.hpp"

#define alpha 0.95
#define beta 0.05

const int watchdog_timer = 500000; // us

namespace Encoder {

    hw_timer_t *Timer0_Cfg = NULL;

    volatile double present_speed;

    void IRAM_ATTR Ext_INT1_ISR() {
        detachInterrupt(ENC_MOTOR_A_CHA_PIN);
        
        uint64_t T_us = timerReadMicros(Timer0_Cfg);

        // Verificar estado do outro canal para saber qual deles veio primeiro
        int direction = digitalRead(ENC_MOTOR_A_CHB_PIN);
        // Quando canal A deu um sinal de subida, qual é a posição do canal B?
        // Supondo que a velocidade é positiva caso B esteja em 1
        direction = direction == 1? 1:-1;

        present_speed = (double) (direction)*(2*PI)/(12*(T_us*1e-6));

        timerRestart(Timer0_Cfg);

        attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
    }

    void setup() {
        pinMode(ENC_MOTOR_A_CHA_PIN, INPUT);
        pinMode(ENC_MOTOR_A_CHB_PIN, INPUT);
        attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
        // Configure Timer0
        Timer0_Cfg = timerBegin(0, 2, true);
        timerWrite(Timer0_Cfg, 0);
        timerStart(Timer0_Cfg);
    }

    double getMotorSpeeds() {
        if (timerReadMicros(Timer0_Cfg) > watchdog_timer) {
            present_speed = 0;
        }

        return present_speed;
    }
}