/**
 * @file adaptive_controller_with_timer.h
 * @brief Adaptive controller with hardware timer for precise timing
 * 
 * This implementation uses ESP32 hardware timer instead of polling
 * to guarantee consistent sampling period for adaptive control.
 */

#ifndef ADAPTIVE_CONTROLLER_TIMER_H
#define ADAPTIVE_CONTROLLER_TIMER_H

namespace AdaptiveControllerTimer {

    /**
     * @brief Initialize adaptive controller with hardware timer
     * @param period_ms Sampling period in milliseconds (default: 10ms)
     */
    void setup(float period_ms = 10.0f);
    
    /**
     * @brief Set reference values for wheel angular velocities
     * @param omega_L_ref Left wheel reference [rad/s]
     * @param omega_R_ref Right wheel reference [rad/s]
     */
    void setReferences(float omega_L_ref, float omega_R_ref);
    
    /**
     * @brief Stop the control timer
     */
    void stop();
    
    /**
     * @brief Start the control timer
     */
    void start();
    
    /**
     * @brief Check if controller is running
     */
    bool isRunning();
    
    // Getters (same as original)
    float getTheta1Left();
    float getTheta2Left();
    float getTheta1Right();
    float getTheta2Right();
    float getOmegaLeft();
    float getOmegaRight();
    float getErrorLeft();
    float getErrorRight();
    float getControlSignalLeft();
    float getControlSignalRight();
    
    // Timing diagnostics
    float getActualPeriod();  // Returns actual measured period in ms
    float getJitter();        // Returns timing jitter (max - min) in ms

    /**
     * Internal: runs one control step in task context. Exposed for tests.
     * Safe to call from non-ISR context only.
     */
    void runControlStep();
}

#endif
