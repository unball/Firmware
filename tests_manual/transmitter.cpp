







// #include <Arduino.h>
// #include "encoder.hpp"
// #include "motor.hpp"
// #include "config.h"
// #include "robot_config.hpp"
// #include "imu.hpp"
// #include "pid_controller.hpp"
// #include "adaptive_controller.h"
// #include "wifi.hpp"
// #include "control.hpp"
// #include "state_space_controller.hpp"

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
//     // PIDController::setGains(1.02f, 0.1f, 0.00f);
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

//     StateSpaceController::update(v_ref, w_ref);
//     // PIDController::update(v_ref, w_ref);

//     // === Send reference to adaptive controller ===
//     AdaptiveController::setReferences(
//         // ((v_ref - (L/2)*w_ref) / R),
//         // ((v_ref + (L/2)*w_ref) / R)
//         // PIDController::getOmegaLeft(),
//         // PIDController::getOmegaRight()
//         StateSpaceController::getControlLeft(),
//         StateSpaceController::getControlRight()
//     );

//     AdaptiveController::update();

//     // === Collect measurements ===
//     Encoder::vel vel = Encoder::getMotorSpeeds();
//     float omega_L = vel.motorLeft;
//     float omega_R = vel.motorRight;
//     float v = (R / 2.0f) * (omega_R + omega_L);
//     // float w = IMU::get_w();
//     float w_encoders = Encoder::getAngularVelocity(R, L);
//     float w = IMU::get_w_filtered(w_encoders , 0.98f);

//     // === Send feedback (throttled to 50 Hz) ===
//     static unsigned long lastFeedback = 0;
//     if (millis() - lastFeedback >= 10) {
//         Wifi::sendFeedback(
//             v, w,
//             v_ref, w_ref,
//             // ((v_ref - (L/2)*w_ref) / R), ((v_ref + (L/2)*w_ref) / R),
//             // PIDController::getOmegaLeft(),                                                              // omega_ref_L // PIDController::getOmegaRight(),  // omega_ref_R
//             StateSpaceController::getControlLeft(), StateSpaceController::getControlRight(),                                                     //  // omega_ref_R
//             omega_L, omega_R,                                                                           // omega L measured by main, omega R measured by main
//             AdaptiveController::getControlSignalLeft(), AdaptiveController::getControlSignalRight(),    // u_R = theta1_R * r_R - theta2_R * omega_R
//             AdaptiveController::getTheta1Left(), AdaptiveController::getTheta2Left(),
//             AdaptiveController::getTheta1Right(), AdaptiveController::getTheta2Right(), 
//             AdaptiveController::getErrorLeft(), AdaptiveController::getErrorRight()
//         );
//         lastFeedback = millis();
//     }

//     // === Print debug ===
//     if (!RobotConfig::getRobotNumber() == 0) {
//         Serial.print(F("v: ")); Serial.print(v, 4);
//         Serial.print(F(" | w: ")); Serial.print(w, 4);
//         Serial.print(F(" || u_L: ")); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
//         Serial.print(F(" | u_R: ")); Serial.print(AdaptiveController::getControlSignalRight(), 2);
//         Serial.print(F(" || omega_L: ")); Serial.print(omega_L, 2);
//         Serial.print(F(" | omega_R: ")); Serial.println(omega_R, 2);
//     } else {
//         Serial.print(F("ref_L: ")); Serial.print(((v_ref - (L/2)*w_ref) / R), 2);
//         Serial.print(F(" | ref_R: ")); Serial.print(((v_ref + (L/2)*w_ref) / R), 2);
//         Serial.print(F(" || LEFT: w_L ")); Serial.print(AdaptiveController::getOmegaLeft(), 3);
//         Serial.print(F(" | u_L: ")); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
//         Serial.print(F(" | theta1_L: ")); Serial.print(AdaptiveController::getTheta1Left(), 3);
//         Serial.print(F(" | theta2_L: ")); Serial.print(AdaptiveController::getTheta2Left(), 3);
//         Serial.print(F(" | e_L: ")); Serial.print(AdaptiveController::getErrorLeft(), 2);
//         Serial.print(F("  ||  RIGHT: w_R: ")); Serial.print(AdaptiveController::getOmegaRight(), 3);
//         Serial.print(F(" | u_R: ")); Serial.print(AdaptiveController::getControlSignalRight(), 2);
//         Serial.print(F(" | theta1_R: ")); Serial.print(AdaptiveController::getTheta1Right(), 3);
//         Serial.print(F(" | theta2_R: ")); Serial.print(AdaptiveController::getTheta2Right(), 3);
//         Serial.print(F(" | e_R: ")); Serial.println(AdaptiveController::getErrorRight(), 2);
//     }
// }






































// #include <Arduino.h>
// #include "encoder.hpp"
// #include "motor.hpp"
// #include "config.h"
// #include "robot_config.hpp"
// #include "imu.hpp"
// #include "pid_controller.hpp"
// #include "adaptive_controller.h"
// #include "wifi.hpp"
// #include "control.hpp"
// #include "state_space_controller.hpp"
// #include <WiFi.h>
// #include <esp_now.h>

// // === Address of the robot ESP32 ===
// // You can use broadcast (ff:ff:ff:ff:ff:ff) or define the robot MAC manually
// uint8_t robotAddress[] = {0x02, 0x55, 0x4E, 0x42, 0x00, 0x00};  // Example for robot 0

// // === Feedback packet structure ===
// typedef struct __attribute__((packed)) {
//     uint32_t timestamp_us;
//     float v_ref;
//     float w_ref;
//     float v;
//     float w;
//     float u_L;
//     float u_R;
//     float omega_L;
//     float omega_R;
//     float w_L;
//     float w_R;
//     float theta1_L;
//     float theta2_L;
//     float theta1_R;
//     float theta2_R;
//     float e_L;
//     float e_R;
// } FeedbackPacket;

// // === Callback when data is received from the robot ===
// void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
//     if (len == sizeof(FeedbackPacket)) {
//         FeedbackPacket p;
//         memcpy(&p, data, sizeof(FeedbackPacket));

//         Serial.printf("t:%lu, v_ref:%.2f, w_ref:%.2f, v:%.2f, w:%.2f, omega_L:%.2f, omega_R:%.2f, u_L:%.2f, u_R:%.2f, ",
//                       p.timestamp_us, p.v_ref, p.w_ref, p.v, p.w, p.omega_L, p.omega_R, p.u_L, p.u_R);
//         Serial.printf("w_L:%.2f, w_R:%.2f, theta1_L:%.2f, theta2_L:%.2f, theta1_R:%.2f, theta2_R:%.2f, e_L:%.2f, e_R:%.2f\n",
//                       p.w_L, p.w_R,
//                       p.theta1_L, p.theta2_L,
//                       p.theta1_R, p.theta2_R,
//                       p.e_L, p.e_R);
//     }
// }

// // === Send command to the robot ===
// void sendCommand(int id, float v, float w) {
//     char buffer[64];  // aumenta para caber floats com separadores
    
//     int16_t v_int =(int16_t)( (v * 32767 )/ 2.0);
//     int16_t w_int = (int16_t)( (w * 32767 )/ 64.0);

//     int32_t checksum = v_int + w_int;
//     int16_t limitedChecksum = (checksum >= 0) 
//       ? (int16_t)(abs(checksum % 32767)) 
//       : -(int16_t)(abs(checksum % 32767));


//     snprintf(buffer, sizeof(buffer), "[%d,%d,%d,%d]", id, v_int, w_int, limitedChecksum);

//     esp_err_t result = esp_now_send(robotAddress, (uint8_t*)buffer, strlen(buffer));

//     if (result != ESP_OK) {
//         Serial.print("[ERROR] Failed to send: ");
//         Serial.println(result);
//     }
// }


// // === Setup ===
// void setup() {
//     Serial.begin(115200);
//     Serial.print("ESP MAC address: ");
//     Serial.println(WiFi.macAddress());
//     WiFi.mode(WIFI_STA);
//     WiFi.disconnect();
//     delay(500);

//     if (esp_now_init() != ESP_OK) {
//         Serial.println("❌ ESP-NOW init failed");
//         return;
//     }
//     esp_err_t error = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

//     esp_now_register_recv_cb(onDataRecv);

//     esp_now_peer_info_t peerInfo = {};
//     memcpy(peerInfo.peer_addr, robotAddress, 6);
//     peerInfo.channel = 14;
//     peerInfo.encrypt = false;

//     if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println("❌ Failed to add peer");
//     }

//     Serial.println("✅ Ready to communicate with robot");
//     #if USE_SIMPLE_TEST
//     Serial.println("[Debug] Using SIMPLE test routine (v only, 0.7s phases)");
//     #else
//     Serial.println("[Debug] Using FULL test routine (straight/circles/squares)");
//     #endif
// }


// // === Simple debug routine (repeating) ===
// // Phases (each 0.7s):
// //  1) v=+0.30, w=0
// //  2) v=0,     w=0
// //  3) v=-0.30, w=0
// //  4) v=0,     w=0
// // Total cycle = 2.8s
// void test_routine_simple(float t, float &v_cmd, float &w_cmd) {
//     const float seg = 0.7f;
//     const float T   = 4.0f * seg; // 2.8s
//     float phase_t = fmodf(t, T);

//     if (phase_t < seg) {
//         v_cmd = 0.30f; w_cmd = 0.0f; // forward
//     } else if (phase_t < 2*seg) {
//         v_cmd = 0.0f;  w_cmd = 0.0f; // stop
//     } else if (phase_t < 3*seg) {
//         v_cmd = -0.30f; w_cmd = 0.0f; // backward
//     } else {
//         v_cmd = 0.0f;  w_cmd = 0.0f; // stop
//     }
// }

// // === Test routine function ===
// void test_routine(float t, float &v_cmd, float &w_cmd) {
//     // === PHASE 1: Straight line tests ===
//     if (t < 0.7f) {
//         // Move forward (0.3 m/s × 0.7s = 21 cm)
//         v_cmd = 0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 1.0f) {
//         // Stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 1.7f) {
//         // Move backward (0.3 m/s × 0.7s = 21 cm)
//         v_cmd = -0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 2.0f) {
//         // Stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
    
//     // === PHASE 2: Circular motion tests ===
//     else if (t < 4.51f) {
//         // Clockwise circle (2π/2.5 ≈ 2.51s)
//         v_cmd = 0.25f;
//         w_cmd = -2.5f;
//     }
//     else if (t < 4.81f) {
//         // Stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 7.32f) {
//         // Counter-clockwise circle
//         v_cmd = 0.25f;
//         w_cmd = 2.5f;
//     }
//     else if (t < 7.62f) {
//         // Stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
    
//     // === PHASE 3: Square path forward ===
//     else if (t < 8.32f) {
//         // Move forward (0.3 m/s × 0.7s = 21 cm)
//         v_cmd = 0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 8.634f) {
//         // Turn 90 degrees
//         v_cmd = 0.0f;
//         w_cmd = 5.0f;
//     }
//     else if (t < 9.334f) {
//         // Move forward
//         v_cmd = 0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 9.648f) {
//         // Turn 90 degrees
//         v_cmd = 0.0f;
//         w_cmd = 5.0f;
//     }
//     else if (t < 10.348f) {
//         // Move forward
//         v_cmd = 0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 10.662f) {
//         // Turn 90 degrees
//         v_cmd = 0.0f;
//         w_cmd = 5.0f;
//     }
//     else if (t < 11.362f) {
//         // Move forward
//         v_cmd = 0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 11.676f) {
//         // Turn 90 degrees
//         v_cmd = 0.0f;
//         w_cmd = 5.0f;
//     }
//     else if (t < 11.976f) {
//         // Stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
    
//     // === PHASE 4: Square path backward ===
//     else if (t < 12.676f) {
//         // Move backward (0.3 m/s × 0.7s = 21 cm)
//         v_cmd = -0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 12.990f) {
//         // Turn -90 degrees
//         v_cmd = 0.0f;
//         w_cmd = -5.0f;
//     }
//     else if (t < 13.690f) {
//         // Move backward
//         v_cmd = -0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 14.004f) {
//         // Turn -90 degrees
//         v_cmd = 0.0f;
//         w_cmd = -5.0f;
//     }
//     else if (t < 14.704f) {
//         // Move backward
//         v_cmd = -0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 15.018f) {
//         // Turn -90 degrees
//         v_cmd = 0.0f;
//         w_cmd = -5.0f;
//     }
//     else if (t < 15.718f) {
//         // Move backward
//         v_cmd = -0.3f;
//         w_cmd = 0.0f;
//     }
//     else if (t < 16.032f) {
//         // Turn -90 degrees
//         v_cmd = 0.0f;
//         w_cmd = -5.0f;
//     }
//     else {
//         // Final stop
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
// }

// void identification_routine(float t, float &v_cmd, float &w_cmd) {
//     // Step input test for system identification
//     if (t < 10.0f) {
//         if (t > 2.0f && t <= 5.0f) {
//             // v_cmd = 0.3f; // Step input for linear velocity
//             w_cmd = 5.0f; // Step input for angular velocity
//         } else {
//             v_cmd = 0.0f;
//             w_cmd = 0.0f;
//         }
//     } else {
//         v_cmd = 0.0f;
//         w_cmd = 0.0f;
//     }
// }

// // === Loop ===
// unsigned long lastSend = 0;
// unsigned long testStartTime = 0;

// // Toggle to run a very simple, repeating debug routine
// #define USE_SIMPLE_TEST false

// void loop() {
//     // Initialize test start time on first run
//     if (testStartTime == 0) {
//         testStartTime = millis();
//     }
    
//     if (millis() - lastSend >= 100) { // Send every 10ms (100 Hz)
//         float t = (millis() - testStartTime) / 1000.0f;
        
//         float v_cmd = 0.0f;
//         float w_cmd = 0.0f;
        
//         // identification_routine(t, v_cmd, w_cmd);
//         // test_routine_simple(t, v_cmd, w_cmd);
//         test_routine(t, v_cmd, w_cmd);
        
//         sendCommand(0, v_cmd, w_cmd);
//         lastSend = millis();
//     }
// }











