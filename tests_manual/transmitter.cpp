

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

        Serial.printf("t:%lu, v_ref:%.2f, w_ref:%.2f, v:%.2f, w:%.2f, omega_L:%.2f, omega_R:%.2f, u_L:%.2f, u_R:%.2f, ",
                      p.timestamp_us, p.v_ref, p.w_ref, p.v, p.w, p.omega_L, p.omega_R, p.u_L, p.u_R);
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

// === Loop: send test command every 100 ms ===
unsigned long lastSend = 0;
void loop() {
    if (millis() - lastSend >= 100) {        
        sendCommand(0, 0.1, 0);
        lastSend = millis();
    }
}
