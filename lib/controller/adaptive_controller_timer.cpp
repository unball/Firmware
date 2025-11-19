/**
 * @file adaptive_controller_with_timer.cpp
 * @brief Adaptive controller with hardware timer implementation
 */

#include "adaptive_controller_timer.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace AdaptiveControllerTimer {

    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    const float pwm_max = 1023.0f;
    const float R = 0.021f;
    const float v_max = 1.429f;
    const float max_safe_pwm = 410.0f;
    const float motor_deadzone_c = 73.0f;
    const float deadzone_threshold = 0.01f;
    
    // Control parameters
    float T = 0.01f;                // Sampling time [s] - set in setup()
    const float tau_m = 0.01f;      // Reference model time constant [s]
    const float gamma_adapt = 0.05f;
    const float sigma = 0.01f;
    const float theta1_limit = 25.0f;
    const float theta2_limit = 25.0f;
    
    float am, bm;  // Will be calculated based on actual T
    
    // ========================================================================
    // STATE VARIABLES (volatile for interrupt safety)
    // ========================================================================
    volatile float theta1_L = (pwm_max * R)/v_max;
    volatile float theta2_L = 0.0f;
    volatile float theta1_R = (pwm_max * R)/v_max;
    volatile float theta2_R = 0.0f;
    
    volatile float omega_L = 0.0f, omega_R = 0.0f;
    volatile float omega_m_L = 0.0f, omega_m_R = 0.0f;
    volatile float u_L = 0.0f, u_R = 0.0f;
    volatile float r_L = 0.0f, r_R = 0.0f;
    
    // ========================================================================
    // TIMER CONTROL
    // ========================================================================
    hw_timer_t *controlTimer = NULL;
    volatile bool running = false;
    static TaskHandle_t controlTaskHandle = NULL;
    
    // Timing diagnostics
    volatile unsigned long last_exec_time = 0;
    volatile unsigned long min_period = 999999999;
    volatile unsigned long max_period = 0;
    volatile unsigned long period_count = 0;
    volatile unsigned long sum_period = 0;
    
    // ========================================================================
    // HELPER FUNCTIONS
    // ========================================================================
    float applyDeadzone(float u_in, float dz_pos, float dz_neg) {
        if (u_in > 0.0f)
            return (u_in > dz_pos) ? u_in : dz_pos;
        else if (u_in < 0.0f)
            return (u_in < -dz_neg) ? u_in : -dz_neg;
        return 0.0f;
    }
    
    // ========================================================================
    // CONTROL: ISR only signals, logic runs in a FreeRTOS task (non-ISR)
    // ========================================================================
    void IRAM_ATTR onControlTimer() {
        // Measure actual timing (for diagnostics)
        unsigned long now = micros();
        if (last_exec_time > 0) {
            unsigned long period = now - last_exec_time;
            sum_period += period;
            period_count++;
            if (period < min_period) min_period = period;
            if (period > max_period) max_period = period;
        }
        last_exec_time = now;

        // Notify control task to run one control step
        BaseType_t xHigherPriorityWoken = pdFALSE;
        if (controlTaskHandle) {
            vTaskNotifyGiveFromISR(controlTaskHandle, &xHigherPriorityWoken);
            if (xHigherPriorityWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }

    static void controlTask(void* pv) {
        (void)pv;
        for (;;) {
            // Wait for timer tick notification
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // Run one control step in task context
            AdaptiveControllerTimer::runControlStep();
        }
    }

    void runControlStep() {
        // Get wheel speeds
        Encoder::vel vel = Encoder::getMotorSpeeds();
        omega_L = vel.motorLeft;
        omega_R = vel.motorRight;

        // RIGHT WHEEL CONTROL
        omega_m_R = am * omega_m_R + bm * r_R;
        float e_R = omega_R - omega_m_R;
        if (fabs(e_R) > deadzone_threshold) {
            float delta_theta1_R = -T * (gamma_adapt * r_R * e_R + sigma * fabs(e_R) * theta1_R);
            float delta_theta2_R =  T * (gamma_adapt * omega_R * e_R - sigma * fabs(e_R) * theta2_R);
            if (!((theta1_R >= theta1_limit && delta_theta1_R > 0) || 
                  (theta1_R <= -theta1_limit && delta_theta1_R < 0)))
                theta1_R += delta_theta1_R;
            if (!((theta2_R >= theta2_limit && delta_theta2_R > 0) || 
                  (theta2_R <= -theta2_limit && delta_theta2_R < 0)))
                theta2_R += delta_theta2_R;
        }
        u_R = theta1_R * r_R - theta2_R * omega_R;
        u_R = constrain(u_R, (r_R >= 0 ? 0.0f : -max_safe_pwm), (r_R >= 0 ? max_safe_pwm : 0.0f));
        {
            float u_R_adj = applyDeadzone(u_R, motor_deadzone_c, motor_deadzone_c);
            Motor::move(MOTOR_RIGHT, u_R_adj);
        }

        // LEFT WHEEL CONTROL
        omega_m_L = am * omega_m_L + bm * r_L;
        float e_L = omega_L - omega_m_L;
        if (fabs(e_L) > deadzone_threshold) {
            float delta_theta1_L = -T * (gamma_adapt * r_L * e_L + sigma * fabs(e_L) * theta1_L);
            float delta_theta2_L =  T * (gamma_adapt * omega_L * e_L - sigma * fabs(e_L) * theta2_L);
            if (!((theta1_L >= theta1_limit && delta_theta1_L > 0) || 
                  (theta1_L <= -theta1_limit && delta_theta1_L < 0)))
                theta1_L += delta_theta1_L;
            if (!((theta2_L >= theta2_limit && delta_theta2_L > 0) || 
                  (theta2_L <= -theta2_limit && delta_theta2_L < 0)))
                theta2_L += delta_theta2_L;
        }
        u_L = theta1_L * r_L - theta2_L * omega_L;
        u_L = constrain(u_L, (r_L >= 0 ? 0.0f : -max_safe_pwm), (r_L >= 0 ? max_safe_pwm : 0.0f));
        {
            float u_L_adj = applyDeadzone(u_L, motor_deadzone_c, motor_deadzone_c);
            Motor::move(MOTOR_LEFT, u_L_adj);
        }
    }
    
    // ========================================================================
    // PUBLIC INTERFACE
    // ========================================================================
    
    void setup(float period_ms) {
        T = period_ms / 1000.0f;  // Convert to seconds
        
        // Calculate discrete-time model parameters
        am = exp(-T / tau_m);
        bm = 1.0f - am;
        
    // Configure hardware timer
        // Timer 0, prescaler 80 → 1 MHz (1 microsecond resolution)
        controlTimer = timerBegin(0, 80, true);
        timerAttachInterrupt(controlTimer, &onControlTimer, true);
        
        // Set period (in microseconds)
        unsigned long period_us = (unsigned long)(period_ms * 1000);
        timerAlarmWrite(controlTimer, period_us, true);  // Auto-reload
        
        // Initialize timing diagnostics
        last_exec_time = 0;
        min_period = 999999999;
        max_period = 0;
        period_count = 0;
        sum_period = 0;
        
        // Create control task (runs control step in task context)
        if (controlTaskHandle == NULL) {
            xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, 3, &controlTaskHandle, 1);
        }

        // Start timer
        timerAlarmEnable(controlTimer);
        running = true;
        
        Serial.printf("[AdaptiveControllerTimer] Initialized with T=%.3f ms\n", period_ms);
        Serial.printf("  am=%.6f, bm=%.6f\n", am, bm);
    }
    
    void setReferences(float omega_L_ref, float omega_R_ref) {
        // Safe write to volatile variables
        noInterrupts();
        r_L = omega_L_ref;
        r_R = omega_R_ref;
        interrupts();
    }
    
    void stop() {
        if (controlTimer != NULL && running) {
            timerAlarmDisable(controlTimer);
            running = false;
            Motor::stop();
        }
    }
    
    void start() {
        if (controlTimer != NULL && !running) {
            timerAlarmEnable(controlTimer);
            running = true;
        }
    }
    
    bool isRunning() {
        return running;
    }
    
    // Getters (safe read from volatile variables)
    float getTheta1Left()  { 
        noInterrupts();
        float val = theta1_L;
        interrupts();
        return val;
    }
    
    float getTheta2Left()  { 
        noInterrupts();
        float val = theta2_L;
        interrupts();
        return val;
    }
    
    float getTheta1Right() { 
        noInterrupts();
        float val = theta1_R;
        interrupts();
        return val;
    }
    
    float getTheta2Right() { 
        noInterrupts();
        float val = theta2_R;
        interrupts();
        return val;
    }
    
    float getOmegaLeft()  { 
        noInterrupts();
        float val = omega_L;
        interrupts();
        return val;
    }
    
    float getOmegaRight() { 
        noInterrupts();
        float val = omega_R;
        interrupts();
        return val;
    }
    
    float getErrorLeft()  { 
        noInterrupts();
        float val = omega_L - omega_m_L;
        interrupts();
        return val;
    }
    
    float getErrorRight() { 
        noInterrupts();
        float val = omega_R - omega_m_R;
        interrupts();
        return val;
    }
    
    float getControlSignalLeft()  { 
        noInterrupts();
        float val = u_L;
        interrupts();
        return val;
    }
    
    float getControlSignalRight() { 
        noInterrupts();
        float val = u_R;
        interrupts();
        return val;
    }
    
    // Timing diagnostics
    float getActualPeriod() {
        if (period_count == 0) return 0;
        noInterrupts();
        float avg = (float)sum_period / period_count / 1000.0f;  // Convert to ms
        interrupts();
        return avg;
    }
    
    float getJitter() {
        noInterrupts();
        float jitter = (max_period - min_period) / 1000.0f;  // Convert to ms
        interrupts();
        return jitter;
    }
}
