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






// === State and reference ===
float v_ref = 0.0f, w_ref = 0.0f;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    IMU::setup();
    RobotConfig::setup();
    Wifi::setup(RobotConfig::getRobotNumber());
    // PIDController::setGains(2.0f, 0.1f, 0.001f);
    PIDController::setGains(1.02f, 0.1f, 0.00f);
    // PIDController::setGains(0.4683f, 0.0f, 0.0282);

    Serial.println("State-space control initialized.");
    Serial.print("Robot Number: ");Serial.println(RobotConfig::getRobotNumber());
    // Initialize motors, encoders, etc. here
}

void loop() {
    // Se for o robô 1, executa Twiddle ao invés da rotina normal
    if (RobotConfig::getRobotNumber() == 1) {
        
        Serial.println("Starting Twiddle algorithm...");
        delay(1000);
        PIDController::twiddle();
        Motor::stop();
        while (1);
        return;
    }

    static int16_t v_int;
    static int16_t w_int;

    // === Receive references from Wi-Fi ===
    Wifi::receiveData(&v_int, &w_int);

    v_ref = ((float)v_int) * 2.0f / 32767;
    w_ref = ((float)w_int) * 64.0f / 32767;

    // // === Update state-space controller (runs every T internally) ===
    // StateSpaceController::update(v_ref, w_ref);

    PIDController::update(v_ref, w_ref);

    // === Send reference to adaptive controller ===
    AdaptiveController::setReferences(
        PIDController::getOmegaLeft(),
        PIDController::getOmegaRight()
    );

    //     // === Send reference to adaptive controller ===
    // AdaptiveController::setReferences(
    //     ((v_ref - (L/2)*w_ref) / R),
    //     ((v_ref + (L/2)*w_ref) / R)
    // );

    AdaptiveController::update();

    // === Collect measurements ===
    Encoder::vel vel = Encoder::getMotorSpeeds();
    float omega_L = vel.motorLeft;
    float omega_R = vel.motorRight;
    float v = (R / 2.0f) * (omega_R + omega_L);
    float w = IMU::get_w();

    // === Send feedback ===
    Wifi::sendFeedback(
        v, w,
        v_ref, w_ref,
        PIDController::getOmegaLeft(),
        PIDController::getOmegaRight(),
        omega_L, omega_R,
        AdaptiveController::getOmegaLeft(),
        AdaptiveController::getOmegaRight(),
        AdaptiveController::getTheta1Left(),
        AdaptiveController::getTheta2Left(),
        AdaptiveController::getTheta1Right(),
        AdaptiveController::getTheta2Right(),
        AdaptiveController::getErrorLeft(),
        AdaptiveController::getErrorRight()
    );

    // === Print debug ===
    if (RobotConfig::getRobotNumber() == 0) {
        Serial.print("v: "); Serial.print(v, 4);
        Serial.print(" | w: "); Serial.print(w, 4);
        Serial.print(" || u_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
        Serial.print(" | u_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
        Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
        Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
    } else {
        Serial.print("ref_L: "); Serial.print(StateSpaceController::getControlLeft(), 2);
        Serial.print(" | ref_R: "); Serial.print(StateSpaceController::getControlRight(), 2);
        Serial.print(" || LEFT: w_L "); Serial.print(AdaptiveController::getOmegaLeft(), 3);
        Serial.print(" | u_L: "); Serial.print(AdaptiveController::getControlSignalLeft(), 2);
        Serial.print(" | theta1_L: "); Serial.print(AdaptiveController::getTheta1Left(), 3);
        Serial.print(" | theta2_L: "); Serial.print(AdaptiveController::getTheta2Left(), 3);
        Serial.print(" | e_L: "); Serial.print(AdaptiveController::getErrorLeft(), 2);
        Serial.print("  ||  RIGHT: w_R: "); Serial.print(AdaptiveController::getOmegaRight(), 3);
        Serial.print(" | u_R: "); Serial.print(AdaptiveController::getControlSignalRight(), 2);
        Serial.print(" | theta1_R: "); Serial.print(AdaptiveController::getTheta1Right(), 3);
        Serial.print(" | theta2_R: "); Serial.print(AdaptiveController::getTheta2Right(), 3);
        Serial.print(" | e_R: "); Serial.println(AdaptiveController::getErrorRight(), 2);
    }
}














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

//         Serial.printf("t:%lu, v_ref:%.6f, w_ref:%.6f, v:%.6f, w:%.1f, omega_L:%.4f, omega_R:%.1f, u_L:%.4f, u_R:%.4f, ",
//                       p.timestamp_us, p.v_ref, p.w_ref, p.v, p.w, p.omega_L, p.omega_R, p.u_L, p.u_R);
//         Serial.printf("w_L:%.4f, w_R:%.1f, theta1_L:%.1f, theta2_L:%.1f, theta1_R:%.1f, theta2_R:%.1f, e_L:%.1f, e_R:%.1f\n",
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
// }

// // === Loop: send test command every 100 ms ===
// unsigned long lastSend = 0;
// void loop() {
//     if (millis() - lastSend >= 100) {
//         // Calcula tempo atual em segundos
//         float t = millis() / 1000.0f;

//         // Parâmetros da onda quadrada
//         float period = 4.0f;       // segundos
//         float amplitude = 0.2f;    // valor máximo da referência (v_ref)

//         // Calcula fase e gera a onda
//         float phase = fmod(t, period);
//         float v_ref = (phase < period / 2.0f) ? amplitude : -amplitude;

//         sendCommand(0, v_ref, 0.0);  // id = 0, v = onda, w = 0
//         lastSend = millis();
//     }
// }






