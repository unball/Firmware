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








// // === State and reference ===
// float v_ref = 0.0f, w_ref = 0.0f;

// void setup() {
//     Serial.begin(115200);
//     Encoder::setup();
//     Motor::setup();
//     IMU::setup();
//     RobotConfig::setup();
//     Wifi::setup(RobotConfig::getRobotNumber());
//     // PIDController::setGains(2.0f, 0.1f, 0.001f);
//     PIDController::setGains(1.02f, 0.1f, 0.00f);
//     // PIDController::setGains(0.4683f, 0.0f, 0.0282);

//     Serial.println("State-space control initialized.");
//     Serial.print("Robot Number: ");Serial.println(RobotConfig::getRobotNumber());
//     // Initialize motors, encoders, etc. here
// }

// void loop() {
//     // Se for o robô 1, executa Twiddle ao invés da rotina normal
//     if (RobotConfig::getRobotNumber() == 1) {
        
//         Serial.println("Starting Twiddle algorithm...");
//         delay(1000);
//         PIDController::twiddle();
//         Motor::stop();
//         while (1);
//         return;
//     }

//     static int16_t v_int;
//     static int16_t w_int;

//     // === Receive references from Wi-Fi ===
//     Wifi::receiveData(&v_int, &w_int);

//     v_ref = ((float)v_int) * 2.0f / 32767;
//     w_ref = ((float)w_int) * 64.0f / 32767;

//     // // === Update state-space controller (runs every T internally) ===
//     // StateSpaceController::update(v_ref, w_ref);

//     PIDController::update(v_ref, w_ref);

//     // === Send reference to adaptive controller ===
//     AdaptiveController::setReferences(
//         PIDController::getOmegaLeft(),
//         PIDController::getOmegaRight()
//     );

//     //     // === Send reference to adaptive controller ===
//     // AdaptiveController::setReferences(
//     //     ((v_ref - (L/2)*w_ref) / R),
//     //     ((v_ref + (L/2)*w_ref) / R)
//     // );

//     AdaptiveController::update();

//     // === Collect measurements ===
//     Encoder::vel vel = Encoder::getMotorSpeeds();
//     float omega_L = vel.motorLeft;
//     float omega_R = vel.motorRight;
//     float v = (R / 2.0f) * (omega_R + omega_L);
//     float w = IMU::get_w();

//     // === Send feedback ===
//     Wifi::sendFeedback(
//         v, w,
//         v_ref, w_ref,
//         PIDController::getOmegaLeft(),
//         PIDController::getOmegaRight(),
//         omega_L, omega_R,
//         AdaptiveController::getOmegaLeft(),
//         AdaptiveController::getOmegaRight(),
//         AdaptiveController::getTheta1Left(),
//         AdaptiveController::getTheta2Left(),
//         AdaptiveController::getTheta1Right(),
//         AdaptiveController::getTheta2Right(),
//         AdaptiveController::getErrorLeft(),
//         AdaptiveController::getErrorRight()
//     );

//     // === Print debug ===
//     if (RobotConfig::getRobotNumber() == 0) {
//         Serial.print("v: "); Serial.print(v, 4);
//         Serial.print(" | w: "); Serial.print(w, 4);
//         Serial.print(" || u_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
//         Serial.print(" | u_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
//         Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
//         Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
//     } else {
//         Serial.print("ref_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
//         Serial.print(" | ref_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
//         Serial.print(" || LEFT: w_L "); Serial.print(AdaptiveController::getOmegaLeft(), 3);
//         Serial.print(" | u_L: "); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
//         Serial.print(" | theta1_L: "); Serial.print(AdaptiveController::getTheta1Left(), 3);
//         Serial.print(" | theta2_L: "); Serial.print(AdaptiveController::getTheta2Left(), 3);
//         Serial.print(" | e_L: "); Serial.print(AdaptiveController::getErrorLeft(), 2);
//         Serial.print("  ||  RIGHT: w_R: "); Serial.print(AdaptiveController::getOmegaRight(), 3);
//         Serial.print(" | u_R: "); Serial.print(AdaptiveController::getControlSignalRight(), 2);
//         Serial.print(" | theta1_R: "); Serial.print(AdaptiveController::getTheta1Right(), 3);
//         Serial.print(" | theta2_R: "); Serial.print(AdaptiveController::getTheta2Right(), 3);
//         Serial.print(" | e_R: "); Serial.println(AdaptiveController::getErrorRight(), 2);
//     }
// }














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

        Serial.printf("t:%lu, v_ref:%.6f, w_ref:%.6f, v:%.6f, w:%.1f, omega_L:%.4f, omega_R:%.1f, u_L:%.4f, u_R:%.4f, ",
                      p.timestamp_us, p.v_ref, p.w_ref, p.v, p.w, p.omega_L, p.omega_R, p.u_L, p.u_R);
        Serial.printf("w_L:%.4f, w_R:%.1f, theta1_L:%.1f, theta2_L:%.1f, theta1_R:%.1f, theta2_R:%.1f, e_L:%.1f, e_R:%.1f\n",
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
}


// === Sequenced tests with command-reversal returns (no odometry, no Serial prints) ===
// Phases:
// 0 idle, 1 in-place (square on v), 2 return of phase 1,
// 3 idle, 4 figure-eight, 5 return of phase 4,
// 6 idle, 7 square path, 8 return of phase 7,
// 9 idle after, 10 done.

void loop() {
    // Persistent state
    static bool inited = false;
    static unsigned long t0 = 0, lastSend = 0;
    static int phase = 0;

    // Command history for reversing
    static const int MAX_SAMPLES = 400;  // 400 samples @100 ms = 40 s
    static float v_hist[MAX_SAMPLES];
    static float w_hist[MAX_SAMPLES];
    static int hist_len = 0;
    static int hist_play_idx = -1;

    // Timing
    const unsigned long SEND_PERIOD_MS = 100;
    const float DUR_IDLE_BEFORE   = 5.0f;   // phase 0
    const float DUR_REF_INPLACE   = 16.0f;  // phase 1
    const float DUR_IDLE_BETWEEN1 = 8.0f;   // phase 3
    const float DUR_FIGURE_EIGHT  = 28.0f;  // phase 4
    const float DUR_IDLE_BETWEEN2 = 8.0f;   // phase 6
    const float DUR_SQUARE_PATH   = 20.0f;  // phase 7
    const float DUR_IDLE_AFTER    = 8.0f;   // phase 9

    // Limits
    const float V_MAX   = 0.35f;
    const float W_MAX   = 3.0f;
    const float SCALE_V = 0.6f;
    const float SCALE_W = 0.8f;

    auto clampf = [](float x, float a, float b){ return x < a ? a : (x > b ? b : x); };

    if (!inited) {
        inited = true;
        t0 = millis();
        lastSend = 0;
        phase = 0;
        hist_len = 0;
        hist_play_idx = -1;
    }

    unsigned long now = millis();
    if (now - lastSend < SEND_PERIOD_MS) return;

    float tg = (now - t0) / 1000.0f;

    // Phase schedule
    float acc = 0.0f;
    int newPhase = phase;
    if      (tg < (acc += DUR_IDLE_BEFORE))               newPhase = 0;
    else if (tg < (acc += DUR_REF_INPLACE))               newPhase = 1;
    else if (tg < (acc += 0.0f))                          newPhase = 2;
    else if (tg < (acc += DUR_IDLE_BETWEEN1))             newPhase = 3;
    else if (tg < (acc += DUR_FIGURE_EIGHT))              newPhase = 4;
    else if (tg < (acc += 0.0f))                          newPhase = 5;
    else if (tg < (acc += DUR_IDLE_BETWEEN2))             newPhase = 6;
    else if (tg < (acc += DUR_SQUARE_PATH))               newPhase = 7;
    else if (tg < (acc += 0.0f))                          newPhase = 8;
    else if (tg < (acc += DUR_IDLE_AFTER))                newPhase = 9;
    else                                                  newPhase = 10;

    static float phaseStartTime = 0.0f;
    if (newPhase != phase) {
        phase = newPhase;
        phaseStartTime = tg;

        if (phase == 1 || phase == 4 || phase == 7) {
            hist_len = 0;
            hist_play_idx = -1;
        }
        if (phase == 2 || phase == 5 || phase == 8) {
            hist_play_idx = hist_len - 1;
        }
    }

    float v_ref = 0.0f;
    float w_ref = 0.0f;

    switch (phase) {
        case 1: { // In-place: square wave
            float t_local = tg - DUR_IDLE_BEFORE;
            const float period = 3.0f;
            const float amp_v  = 0.12f;
            float ph = fmodf(t_local, period);
            v_ref = (ph < 0.5f * period) ? amp_v : -amp_v;
            w_ref = 0.0f;
            v_ref = clampf(v_ref * SCALE_V, -V_MAX, V_MAX);
            w_ref = clampf(w_ref * SCALE_W, -W_MAX, W_MAX);
            if (hist_len < MAX_SAMPLES) {
                v_hist[hist_len] = v_ref;
                w_hist[hist_len] = w_ref;
                hist_len++;
            }
        } break;

        case 2: { // Return for phase 1
            if (hist_play_idx >= 0) {
                v_ref = -v_hist[hist_play_idx];
                w_ref = -w_hist[hist_play_idx];
                hist_play_idx--;
            }
        } break;

        case 4: { // Figure-eight
            float t_local = tg - (DUR_IDLE_BEFORE + DUR_REF_INPLACE);
            const float v_base = 0.15f;
            const float w_amp  = 1.6f;
            const float w_freq = 0.7f;
            v_ref = v_base;
            w_ref = w_amp * sinf(w_freq * t_local);
            v_ref = clampf(v_ref * SCALE_V, -V_MAX, V_MAX);
            w_ref = clampf(w_ref * SCALE_W, -W_MAX, W_MAX);
            if (hist_len < MAX_SAMPLES) {
                v_hist[hist_len] = v_ref;
                w_hist[hist_len] = w_ref;
                hist_len++;
            }
        } break;

        case 5: { // Return for phase 4
            if (hist_play_idx >= 0) {
                v_ref = -v_hist[hist_play_idx];
                w_ref = -w_hist[hist_play_idx];
                hist_play_idx--;
            }
        } break;

        case 7: { // Square path
            float t_local = tg - (DUR_IDLE_BEFORE + DUR_REF_INPLACE + DUR_IDLE_BETWEEN1 + DUR_FIGURE_EIGHT);
            const float side_cycle = 2.0f; // 1.2 s straight + 0.8 s turn
            float ph = fmodf(t_local, side_cycle);
            if (ph < 1.2f) {
                v_ref = 0.15f;
                w_ref = 0.0f;
            } else {
                v_ref = 0.0f;
                w_ref = 1.2f;
            }
            v_ref = clampf(v_ref * SCALE_V, -V_MAX, V_MAX);
            w_ref = clampf(w_ref * SCALE_W, -W_MAX, W_MAX);
            if (hist_len < MAX_SAMPLES) {
                v_hist[hist_len] = v_ref;
                w_hist[hist_len] = w_ref;
                hist_len++;
            }
        } break;

        case 8: { // Return for phase 7
            if (hist_play_idx >= 0) {
                v_ref = -v_hist[hist_play_idx];
                w_ref = -w_hist[hist_play_idx];
                hist_play_idx--;
            }
        } break;

        default:
            v_ref = 0.0f;
            w_ref = 0.0f;
        break;
    }

    v_ref = clampf(v_ref, -V_MAX, V_MAX);
    w_ref = clampf(w_ref, -W_MAX, W_MAX);
    sendCommand(0, v_ref, w_ref);

    lastSend = now;
}
