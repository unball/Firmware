/**
 * @file test_timing_jitter.cpp
 * @brief Test to measure timing jitter: Polling vs Timer/Interrupt
 * 
 * This test compares the actual timing performance of:
 * 1. Polling approach (check millis() in loop)
 * 2. Timer interrupt approach (hardware timer)
 * 
 * Upload this to your ESP32 and observe Serial Monitor.
 */

#include <Arduino.h>

// ============================================================================
// TEST CONFIGURATION
// ============================================================================
#define TEST_DURATION_MS 10000  // Run test for 10 seconds
#define TARGET_PERIOD_US 10000  // Target: 10ms = 10000us (100 Hz)

// Choose test mode
enum TestMode {
    POLLING_MODE,
    TIMER_MODE
};

TestMode currentMode = POLLING_MODE;  // Change to TIMER_MODE to test timer

// ============================================================================
// STATISTICS
// ============================================================================
struct TimingStats {
    unsigned long count;
    unsigned long sum_period;
    unsigned long min_period;
    unsigned long max_period;
    unsigned long sum_squared_error;
} stats;

// ============================================================================
// POLLING APPROACH (Current implementation)
// ============================================================================
unsigned long last_time_polling = 0;
const unsigned long TARGET_PERIOD_MS = TARGET_PERIOD_US / 1000;

void updatePolling() {
    unsigned long now = millis();
    
    // Check if period elapsed
    if (now - last_time_polling < TARGET_PERIOD_MS) {
        return;  // Not time yet
    }
    
    // Measure actual period
    unsigned long actual_period = (now - last_time_polling) * 1000;  // Convert to us
    last_time_polling = now;
    
    // Update statistics
    if (stats.count > 0) {  // Skip first measurement (no previous reference)
        stats.sum_period += actual_period;
        if (actual_period < stats.min_period) stats.min_period = actual_period;
        if (actual_period > stats.max_period) stats.max_period = actual_period;
        
        long error = (long)actual_period - (long)TARGET_PERIOD_US;
        stats.sum_squared_error += error * error;
    }
    
    stats.count++;
}

// ============================================================================
// TIMER INTERRUPT APPROACH (Alternative implementation)
// ============================================================================
hw_timer_t *controlTimer = NULL;
volatile unsigned long last_time_timer = 0;

void IRAM_ATTR onControlTimer() {
    unsigned long now = micros();
    
    // Measure actual period
    unsigned long actual_period = now - last_time_timer;
    last_time_timer = now;
    
    // Update statistics
    if (stats.count > 0) {  // Skip first measurement
        stats.sum_period += actual_period;
        if (actual_period < stats.min_period) stats.min_period = actual_period;
        if (actual_period > stats.max_period) stats.max_period = actual_period;
        
        long error = (long)actual_period - (long)TARGET_PERIOD_US;
        stats.sum_squared_error += error * error;
    }
    
    stats.count++;
}

// ============================================================================
// SETUP AND LOOP
// ============================================================================

void resetStats() {
    stats.count = 0;
    stats.sum_period = 0;
    stats.min_period = 999999999;
    stats.max_period = 0;
    stats.sum_squared_error = 0;
}

void printStats(const char* mode_name) {
    float avg_period = (float)stats.sum_period / stats.count;
    float jitter = stats.max_period - stats.min_period;
    float rmse = sqrt((float)stats.sum_squared_error / stats.count);
    
    Serial.println("\n========================================");
    Serial.printf("MODE: %s\n", mode_name);
    Serial.println("========================================");
    Serial.printf("Samples: %lu\n", stats.count);
    Serial.printf("Target Period: %.3f ms (%.1f Hz)\n", 
                  TARGET_PERIOD_US / 1000.0, 1000000.0 / TARGET_PERIOD_US);
    Serial.println("\nTIMING PERFORMANCE:");
    Serial.printf("  Average Period: %.3f ms (%.1f Hz)\n", 
                  avg_period / 1000.0, 1000000.0 / avg_period);
    Serial.printf("  Min Period: %.3f ms\n", stats.min_period / 1000.0);
    Serial.printf("  Max Period: %.3f ms\n", stats.max_period / 1000.0);
    Serial.printf("  Jitter (max-min): %.3f ms (%.1f%%)\n", 
                  jitter / 1000.0, 100.0 * jitter / TARGET_PERIOD_US);
    Serial.printf("  RMSE: %.3f us\n", rmse);
    
    Serial.println("\nIMPACT ON CONTROL:");
    
    // Calculate impact on adaptive control
    float period_error_percent = 100.0 * (avg_period - TARGET_PERIOD_US) / TARGET_PERIOD_US;
    Serial.printf("  Period Error: %.2f%%\n", period_error_percent);
    
    if (fabs(period_error_percent) > 5.0) {
        Serial.println("  ⚠️  WARNING: >5% error affects adaptive gains!");
    }
    
    if (jitter > TARGET_PERIOD_US * 0.1) {
        Serial.println("  ⚠️  WARNING: High jitter (>10%) may cause instability!");
    }
    
    if (jitter < TARGET_PERIOD_US * 0.01) {
        Serial.println("  ✅ Excellent: Low jitter (<1%)");
    } else if (jitter < TARGET_PERIOD_US * 0.05) {
        Serial.println("  ✅ Good: Moderate jitter (<5%)");
    } else {
        Serial.println("  ❌ Poor: High jitter may affect control quality");
    }
    
    Serial.println("========================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║     TIMING JITTER TEST: Polling vs Timer Interrupt        ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    
    resetStats();
    
    if (currentMode == POLLING_MODE) {
        Serial.println("\n>>> Testing POLLING MODE (current implementation)");
        Serial.println("    Checking millis() in loop() and skipping if not time yet");
        Serial.println("\n    Starting test...\n");
        last_time_polling = millis();
        
    } else {
        Serial.println("\n>>> Testing TIMER INTERRUPT MODE (alternative)");
        Serial.println("    Using hardware timer for precise timing");
        Serial.println("\n    Starting test...\n");
        
        // Configure timer
        controlTimer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz (1us resolution)
        timerAttachInterrupt(controlTimer, &onControlTimer, true);
        timerAlarmWrite(controlTimer, TARGET_PERIOD_US, true);  // Period in microseconds
        timerAlarmEnable(controlTimer);
        
        last_time_timer = micros();
    }
}

void loop() {
    static unsigned long test_start = millis();
    
    if (currentMode == POLLING_MODE) {
        // Call update function (like in your main.cpp)
        updatePolling();
        
        // Simulate other work in loop (typical scenario)
        delayMicroseconds(50);  // Simulates WiFi, Serial, etc.
    }
    
    // Check if test is complete
    if (millis() - test_start >= TEST_DURATION_MS) {
        if (currentMode == POLLING_MODE) {
            printStats("POLLING (current)");
        } else {
            printStats("TIMER INTERRUPT (alternative)");
        }
        
        Serial.println("\n📊 RECOMMENDATION:");
        if (currentMode == POLLING_MODE) {
            Serial.println("  For adaptive control, consider using Timer Interrupt for:");
            Serial.println("    • More consistent period (less jitter)");
            Serial.println("    • Predictable timing for adaptation gains");
            Serial.println("    • Better control performance");
            Serial.println("\n  To test Timer mode, change: currentMode = TIMER_MODE");
        } else {
            Serial.println("  Timer Interrupt provides superior timing precision!");
            Serial.println("  Recommended for adaptive control applications.");
        }
        
        Serial.println("\n  Re-upload with other mode to compare.");
        Serial.println("  Test will restart in 10 seconds...\n");
        
        delay(10000);
        
        // Restart test
        resetStats();
        test_start = millis();
        if (currentMode == POLLING_MODE) {
            last_time_polling = millis();
        } else {
            last_time_timer = micros();
        }
    }
}

/* ============================================================================
   EXPECTED RESULTS
   ============================================================================
   
   POLLING MODE (Current):
   -----------------------
   Average Period: ~10.0-10.2 ms
   Jitter: 1-5 ms (10-50% of period!)
   RMSE: 200-1000 us
   
   → Acceptable for: Simple PID, slow systems
   → NOT IDEAL for: Adaptive control, fast systems
   
   
   TIMER INTERRUPT MODE (Alternative):
   -----------------------------------
   Average Period: ~10.00 ms
   Jitter: 0.001-0.01 ms (<0.1% of period!)
   RMSE: 1-10 us
   
   → Excellent for: Adaptive control, fast dynamics
   → Required for: High-performance control
   
   
   WHY IT MATTERS FOR ADAPTIVE CONTROL:
   ------------------------------------
   
   1. ADAPTATION RATE:
      delta_theta = -T * (gamma * r * e + ...)
                     ↑
      If T varies 20%, adaptation rate varies 20%!
      
   2. REFERENCE MODEL:
      omega_m = am * omega_m + bm * r
      where am = exp(-T/tau_m), bm = 1 - am
      
      If T inconsistent → model inconsistent → poor tracking
   
   3. STABILITY:
      Adaptive control theory assumes fixed sampling time.
      Variable T can lead to parameter drift or instability.
   
   ============================================================================
*/
