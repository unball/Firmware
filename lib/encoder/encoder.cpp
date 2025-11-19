


#include "encoder.hpp"

const int watchdog_timer = 500000; // us
constexpr uint64_t MIN_VALID_T_US = 5000;    // Max RPM = 650 -> T_us = 7700us
constexpr float ALPHA = 0.3f;  // Low-pass filter coefficient (0 < alpha <= 1)
                               // Lower values = more smoothing, higher values = faster response

namespace Encoder {

    hw_timer_t *Timer0_Cfg = NULL;
    hw_timer_t *Timer1_Cfg = NULL;

    volatile float present_speed_A;
    volatile float present_speed_B;
    volatile float filtered_speed_A;
    volatile float filtered_speed_B;

    void IRAM_ATTR Ext_INT1_ISR() {
        detachInterrupt(ENC_MOTOR_A_CHA_PIN);
        
        uint64_t T_us = timerReadMicros(Timer0_Cfg);


        bool A = digitalRead(ENC_MOTOR_A_CHA_PIN);
        bool B = digitalRead(ENC_MOTOR_A_CHB_PIN);
        int direction = (A == B) ? -1 : 1;

        // Filters out unrealistically short pulses caused by noise or startup
        if (T_us >= MIN_VALID_T_US) {
            float raw_speed = (direction)*(2*PI)/(12*(T_us*1e-6));
            // Apply 1st order IIR low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            filtered_speed_A = ALPHA * raw_speed + (1 - ALPHA) * filtered_speed_A;
            present_speed_A = filtered_speed_A;
        }

        timerRestart(Timer0_Cfg);

        attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
    }

    void IRAM_ATTR Ext_INT2_ISR() {
        detachInterrupt(ENC_MOTOR_B_CHA_PIN);
        
        uint64_t T_us = timerReadMicros(Timer1_Cfg);

        bool A = digitalRead(ENC_MOTOR_B_CHA_PIN);
        bool B = digitalRead(ENC_MOTOR_B_CHB_PIN);
        int direction = (A == B) ? -1 : 1;
        
        // Filters out unrealistically short pulses caused by noise or startup.
        if (T_us >= MIN_VALID_T_US) {
            float raw_speed = (direction)*(2*PI)/(12*(T_us*1e-6));
            // Apply 1st order IIR low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            filtered_speed_B = ALPHA * raw_speed + (1 - ALPHA) * filtered_speed_B;
            present_speed_B = filtered_speed_B;
        }

        timerRestart(Timer1_Cfg);

        attachInterrupt(ENC_MOTOR_B_CHA_PIN, Ext_INT2_ISR, RISING);
    }

    void setup() {
        present_speed_A = 0;
        present_speed_B = 0;
        filtered_speed_A = 0;
        filtered_speed_B = 0;

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

        result.motorRight = present_speed_A;
        result.motorLeft = present_speed_B;

        return result;
    }

    float getAngularVelocity(float R, float L) {
        vel speeds = getMotorSpeeds();
        
        // Convert angular velocities of wheels to linear velocities
        float v_right = speeds.motorRight * R;
        float v_left = speeds.motorLeft * R;
        
        // Calculate robot's angular velocity: w = (v_right - v_left) / L
        return (v_right - v_left) / L;
    }
}