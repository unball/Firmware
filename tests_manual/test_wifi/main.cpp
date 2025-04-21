#include <Arduino.h>
#include "wifi.hpp"
#include "robot_config.hpp"

bool initOK = true;
bool messageReceived = false;
int16_t v = 0, w = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("[TEST START] Wi-Fi (ESP-NOW)");

    // Etapas de inicialização (avaliadas individualmente)
    if (!WiFi.mode(WIFI_STA)) {
        Serial.println("❌ Error setting Wi-Fi mode.\n[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    if (!WiFi.disconnect()) {
        Serial.println("❌ Error disconnecting Wi-Fi.\n[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ Error initializing ESP-NOW.\n[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    if (!WiFi.setTxPower(WIFI_POWER_19_5dBm)) {
        Serial.println("❌ Error setting TX power.\n[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    esp_err_t err = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("❌ Error setting Wi-Fi channel: ");
        switch (err) {
        case ESP_ERR_WIFI_NOT_INIT:
            Serial.println("Wi-Fi not initialized.");
            break;
        case ESP_ERR_WIFI_IF:
            Serial.println("Invalid Wi-Fi interface.");
            break;
        case ESP_ERR_INVALID_ARG:
            Serial.println("Invalid argument (channel or secondary).");
            break;
        default:
            Serial.printf("Unknown error code: 0x%X\n", err);
        }
        Serial.println("[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    esp_err_t cbResult = esp_now_register_recv_cb(esp_now_recv_cb_t(Wifi::OnDataRecv));
    if (cbResult != ESP_OK) {
    Serial.print("❌ Error registering receive callback: ");
    switch (cbResult) {
        case ESP_ERR_ESPNOW_NOT_INIT:
            Serial.println("ESP-NOW not initialized.");
            break;
        case ESP_ERR_ESPNOW_INTERNAL:
            Serial.println("Internal ESP-NOW error.");
            break;
        default:
            Serial.printf("Unknown error code: 0x%X\n", cbResult);
    }
        Serial.println("[TEST RESULT] FAIL");
        initOK = false;
        return;
    }

    // Registra o callback e prepara variáveis
    Wifi::setup(RobotConfig::getRobotNumber());

    Serial.println("✅ ESP-NOW setup completed.");
    Serial.println("Waiting for message...");
}

void loop() {
    if (!initOK)
        return;

    static unsigned long startTime = millis();

    // Testa se chegou alguma mensagem nos últimos N microssegundos
    if (!Wifi::isCommunicationLost() && !messageReceived) {
        Wifi::receiveData(&v, &w);  // pega o valor (pode ser zero!)
        messageReceived = true;
        Serial.printf("✅ Message received: v = %d, w = %d\n", v, w);
        Serial.println("[TEST RESULT] PASS");
    }

    if (millis() - startTime > 5000) {
        if (!messageReceived) {
            Serial.println("❌ No valid message received.\n[TEST RESULT] FAIL");
        }

        while (true) {
            delay(1000);
        }
    }

    delay(50);
}


