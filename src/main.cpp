#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"
#include "robot_config.hpp"
#include "imu.hpp"
#include "adaptive_controller.h"
#include "wifi.hpp"
#include "control.hpp"

// === System parameters ===
// const float R = 0.021f;     // Wheel radius [m]
// const float L = 0.075f;     // Wheelbase [m]
const float T = 0.01f;      // Sampling time [s]
const float u_min = -69.0f; // Control saturation
const float u_max =  69.0f;

// === Gains from MATLAB (example with poles = [0.9, 0.9]) ===
const float K[2][2] = {
    {24.9952, -0.9373},
    {24.9952,  0.9373}
};

const float N[2][2] = {
    {72.1380f, -2.7052},
    {72.1380f,  2.7052}
};

// === State and reference ===
float v = 0.0f, w = 0.0f;                  // Measured linear and angular velocity
float v_ref = 0.0f, w_ref = 0.0f;          // Reference signals
float u_L = 0.0f, u_R = 0.0f;              // Control outputs

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    IMU::setup();
    RobotConfig::setup();
    Wifi::setup(0);
    Serial.println("State-space control initialized.");
    Serial.print("Robot Number: ");Serial.println(RobotConfig::getRobotNumber());
    // Initialize motors, encoders, etc. here
}

void loop() {
    static int16_t v_int;
    static int16_t w_int;

    static unsigned long lastTime = 0;
    if (millis() - lastTime >= T * 1000) {
        lastTime = millis();

        // === Read measured velocities (from encoder/IMU) ===
        Encoder::vel vel = Encoder::getMotorSpeeds();
        float omega_L = vel.motorLeft;
        float omega_R = vel.motorRight;

        Wifi::receiveData(&v_int, &w_int);

        // v_ref = ((float)v_int) * 2.0 / 32767;
        // w_ref  = ((float)w_int) * 64.0 / 32767;
        v_ref = 0.1;
        w_ref  = 0;

        float v = (R / 2.0f) * (omega_R + omega_L);
        // float w = 0.0f;  // velocidade angular do robô
        float w = IMU::get_w(); // usar giroscópio para velocidade angular
        float x[2] = {v, w};
        float r[2] = {v_ref, w_ref};

        // === Compute control: u = -Kx + Nr ===
        float u_unsat[2];
        if (v_ref == 0 && w_ref == 0){
            u_unsat[0] = 0.0f;
            u_unsat[1] = 0.0f;
            Motor::stop();
        } else {

            // for (int i = 0; i < 2; i++) {
            //     u_unsat[i] = 0.0f;
            //     for (int j = 0; j < 2; j++) {
            //         u_unsat[i] += -K[i][j] * x[j] + N[i][j] * r[j];
            //     }
            double eW = w_ref - w;
            double w_control = Control::PID(0, eW);
            Serial.print("w_control: "); Serial.println(w_control, 4);

            u_unsat[0] = (v + (L/2)*w_control) / R;
            u_unsat[1] = (v - (L/2)*w_control) / R;
            
        }

        // === Apply saturation ===
        u_L = constrain(u_unsat[0], u_min, u_max);
        u_R = constrain(u_unsat[1], u_min, u_max);

        // === Send control to motors ===
        // setMotorSpeed(leftMotor, u_L);
        // setMotorSpeed(rightMotor, u_R);
        AdaptiveController::setReferences(
        //    5.0,5.0
         u_L,
         u_R
        );

        AdaptiveController::update();

        Wifi::sendFeedback(
            v, w,
            v_ref, w_ref,
            u_L, u_R,
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
            Serial.print(" || u_L: "); Serial.print(u_L, 2);
            Serial.print(" | u_R: "); Serial.print(u_R, 2);
            Serial.print(" || omega_L: "); Serial.print(omega_L, 2);
            Serial.print(" | omega_R: "); Serial.println(omega_R, 2);
        } else {
            Serial.print("ref_L: "); Serial.print(u_L, 2);
            Serial.print(" | ref_R: "); Serial.print(u_R, 2);
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
//         float amplitude = 0.1f;    // valor máximo da referência (v_ref)

//         // Calcula fase e gera a onda
//         float phase = fmod(t, period);
//         float v_ref = (phase < period / 2.0f) ? amplitude : -amplitude;

//         sendCommand(0, v_ref, 0.0);  // id = 0, v = onda, w = 0
//         lastSend = millis();
//     }
// }






