



#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "robot_config.hpp"
#include "imu.hpp"
#include "pid_controller.hpp"
#include "adaptive_controller.h"
#include "wifi.hpp"
#include "control.hpp"
#include "state_space_controller.hpp"
#include <WiFi.h>
#include <esp_now.h>

// === Address of the robot ESP32 ===
// You can use broadcast (ff:ff:ff:ff:ff:ff) or define the robot MAC manually
uint8_t robotAddress[] = {0x02, 0x55, 0x4E, 0x42, 0x00, 0x00};  // Example for robot 0

// === Feedback packet structure ===
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    float v_ref;
    float w_ref;
    float v;
    float w;
    float u_L;
    float u_R;
    float omega_L;
    float omega_R;
    float w_L;
    float w_R;
    float theta1_L;
    float theta2_L;
    float theta1_R;
    float theta2_R;
    float e_L;
    float e_R;
} FeedbackPacket;

// === Callback when data is received from the robot ===
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(FeedbackPacket)) {
        FeedbackPacket p;
        memcpy(&p, data, sizeof(FeedbackPacket));

        Serial.printf("t:%lu, v_ref:%.2f, w_ref:%.2f, v:%.2f, w:%.2f, u_L:%.2f, u_R:%.2f, omega_L:%.2f, omega_R:%.2f, ",
                  p.timestamp_us, p.v_ref, p.w_ref, p.v, p.w, p.u_L, p.u_R, p.omega_L, p.omega_R);
        Serial.printf("w_L:%.2f, w_R:%.2f, theta1_L:%.2f, theta2_L:%.2f, theta1_R:%.2f, theta2_R:%.2f, e_L:%.2f, e_R:%.2f\n",
                      p.w_L, p.w_R,
                      p.theta1_L, p.theta2_L,
                      p.theta1_R, p.theta2_R,
                      p.e_L, p.e_R);
    }
}

// === Send command to the robot ===
void sendCommand(int id, float v, float w) {
    char buffer[64];  // aumenta para caber floats com separadores
    
    int16_t v_int =(int16_t)( (v * 32767 )/ 2.0);
    int16_t w_int = (int16_t)( (w * 32767 )/ 64.0);

    int32_t checksum = v_int + w_int;
    int16_t limitedChecksum = (checksum >= 0) 
      ? (int16_t)(abs(checksum % 32767)) 
      : -(int16_t)(abs(checksum % 32767));


    snprintf(buffer, sizeof(buffer), "[%d,%d,%d,%d]", id, v_int, w_int, limitedChecksum);

    esp_err_t result = esp_now_send(robotAddress, (uint8_t*)buffer, strlen(buffer));

    if (result != ESP_OK) {
        Serial.print("[ERROR] Failed to send: ");
        Serial.println(result);
    }
}


// === Setup ===
void setup() {
    Serial.begin(115200);
    Serial.print("ESP MAC address: ");
    Serial.println(WiFi.macAddress());
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(500);

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }
    esp_err_t error = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, robotAddress, 6);
    peerInfo.channel = 14;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("❌ Failed to add peer");
    }

    Serial.println("✅ Ready to communicate with robot");
    #if USE_SIMPLE_TEST
    Serial.println("[Debug] Using SIMPLE test routine (v only, 0.7s phases)");
    #else
    Serial.println("[Debug] Using FULL test routine (straight/circles/squares)");
    #endif
}


void test_routine_simple(float t, float &v_cmd, float &w_cmd) {
    const float seg = 0.7f;
    const float T   = 4.0f * seg; // 2.8s
    float phase_t = fmodf(t, T);

    if (phase_t < seg) {
        v_cmd = 0.30f; w_cmd = 0.0f; // forward
    } else if (phase_t < 2*seg) {
        v_cmd = 0.0f;  w_cmd = 0.0f; // stop
    } else if (phase_t < 3*seg) {
        v_cmd = -0.30f; w_cmd = 0.0f; // backward
    } else {
        v_cmd = 0.0f;  w_cmd = 0.0f; // stop
    }
}

// === Test routine function ===
void test_routine(float t, float &v_cmd, float &w_cmd) {
    // === PHASE 1: Straight line tests ===
    if (t < 0.7f) {
        // Move forward (0.3 m/s × 0.7s = 21 cm)
        v_cmd = 0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 1.0f) {
        // Stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
    else if (t < 1.7f) {
        // Move backward (0.3 m/s × 0.7s = 21 cm)
        v_cmd = -0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 2.0f) {
        // Stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
    
    // === PHASE 2: Circular motion tests ===
    else if (t < 4.51f) {
        // Clockwise circle (2π/2.5 ≈ 2.51s)
        v_cmd = 0.25f;
        w_cmd = -2.5f;
    }
    else if (t < 4.81f) {
        // Stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
    else if (t < 7.32f) {
        // Counter-clockwise circle
        v_cmd = 0.25f;
        w_cmd = 2.5f;
    }
    else if (t < 7.62f) {
        // Stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
    
    // === PHASE 3: Square path forward ===
    else if (t < 8.32f) {
        // Move forward (0.3 m/s × 0.7s = 21 cm)
        v_cmd = 0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 8.634f) {
        // Turn 90 degrees
        v_cmd = 0.0f;
        w_cmd = 5.0f;
    }
    else if (t < 9.334f) {
        // Move forward
        v_cmd = 0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 9.648f) {
        // Turn 90 degrees
        v_cmd = 0.0f;
        w_cmd = 5.0f;
    }
    else if (t < 10.348f) {
        // Move forward
        v_cmd = 0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 10.662f) {
        // Turn 90 degrees
        v_cmd = 0.0f;
        w_cmd = 5.0f;
    }
    else if (t < 11.362f) {
        // Move forward
        v_cmd = 0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 11.676f) {
        // Turn 90 degrees
        v_cmd = 0.0f;
        w_cmd = 5.0f;
    }
    else if (t < 11.976f) {
        // Stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
    
    // === PHASE 4: Square path backward ===
    else if (t < 12.676f) {
        // Move backward (0.3 m/s × 0.7s = 21 cm)
        v_cmd = -0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 12.990f) {
        // Turn -90 degrees
        v_cmd = 0.0f;
        w_cmd = -5.0f;
    }
    else if (t < 13.690f) {
        // Move backward
        v_cmd = -0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 14.004f) {
        // Turn -90 degrees
        v_cmd = 0.0f;
        w_cmd = -5.0f;
    }
    else if (t < 14.704f) {
        // Move backward
        v_cmd = -0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 15.018f) {
        // Turn -90 degrees
        v_cmd = 0.0f;
        w_cmd = -5.0f;
    }
    else if (t < 15.718f) {
        // Move backward
        v_cmd = -0.3f;
        w_cmd = 0.0f;
    }
    else if (t < 16.032f) {
        // Turn -90 degrees
        v_cmd = 0.0f;
        w_cmd = -5.0f;
    }
    else {
        // Final stop
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
}

void identification_routine(float t, float &v_cmd, float &w_cmd) {
    // Step input test for system identification
    if (t < 10.0f) {
        if (t > 2.0f && t <= 5.0f) {
            // v_cmd = 0.3f; // Step input for linear velocity
            w_cmd = 5.0f; // Step input for angular velocity
        } else {
            v_cmd = 0.0f;
            w_cmd = 0.0f;
        }
    } else {
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
}

// === TEST 1: MRAC Step in v (forward -> stop -> backward -> stop) ===
void test_mrac_1_step_v(float t, float &v_cmd, float &w_cmd) {
    w_cmd = 0.0f;

    // Phase 1: forward (0 to 3 s)
    if (t < 3.0f) {
        v_cmd = 0.40f;
    }
    // Phase 2: short stop (3 to 3.5 s)
    else if (t < 3.5f) {
        v_cmd = 0.0f;
    }
    // Phase 3: backward (3.5 to 6.5 s)
    else if (t < 6.5f) {
        v_cmd = -0.40f;
    }
    // Phase 4: final stop (6.5 s onward)
    else {
        v_cmd = 0.0f;
    }
}


// === TEST 2: MRAC Step in w (CW -> stop -> CCW -> stop) ===
void test_mrac_2_step_w(float t, float &v_cmd, float &w_cmd) {
    v_cmd = 0.0f;

    // Phase 1: CW rotation (0 to 3.0 s)
    if (t < 3.0f) {
        w_cmd = 5.0f;    // clockwise
    }
    // Phase 2: short stop (3.0 to 3.5 s)
    else if (t < 3.5f) {
        w_cmd = 0.0f;
    }
    // Phase 3: CCW rotation (3.5 to 6.5 s)
    else if (t < 6.5f) {
        w_cmd = -5.0f;   // counter-clockwise
    }
    // Phase 4: final stop (6.5 s onward)
    else {
        w_cmd = 0.0f;
    }
}


// === TEST 3: MRAC Slow Sine in v (w = 0) ===
void test_mrac_3_sine_v(float t, float &v_cmd, float &w_cmd) {
    w_cmd = 0.0f;

    // amplitude 0.15 m/s, offset 0.15 m/s, period ≈ 10 s
    const float A = 0.3f;
    const float offset = 0.3f;
    const float freq = 0.5f; // Hz (period 10s)

    if (t < 10.0f) {
        v_cmd = offset + A * sinf(2.0f * 3.14159f * freq * t);
    } else {
        v_cmd = 0.0f;
    }
}

// === TEST 4: MRAC Constant Circle ===
void test_mrac_4_circle(float t, float &v_cmd, float &w_cmd) {
    // v fixed, w fixed
    if (t < 4.0f) {
        v_cmd = 0.35f;   // moderate translational speed
        w_cmd = 2.5f;    // turn left (counter-clockwise)
    } else if (t < 8.0f) {
        v_cmd = -0.35f;
        w_cmd = 2.5f;
    } else {
        v_cmd = 0.0f;
        w_cmd = 0.0f;
    }
}


// === Loop ===
unsigned long lastSend = 0;
unsigned long testStartTime = 0;

void loop() {
    // Initialize test start time on first run
    if (testStartTime == 0) {
        testStartTime = millis();
    }
    
    if (millis() - lastSend >= 100) { // Send every 10ms (100 Hz)
        float t = (millis() - testStartTime) / 1000.0f;
        
        float v_cmd = 0.0f;
        float w_cmd = 0.0f;
        
        // identification_routine(t, v_cmd, w_cmd);
        // test_routine_simple(t, v_cmd, w_cmd);
        // test_routine(t, v_cmd, w_cmd);

        // test_mrac_1_step_v(t, v_cmd, w_cmd);
        // test_mrac_2_step_w(t, v_cmd, w_cmd);
        // test_mrac_3_sine_v(t, v_cmd, w_cmd);
        test_mrac_4_circle(t, v_cmd, w_cmd);

        
        sendCommand(0, v_cmd, w_cmd);
        lastSend = millis();
    }
}