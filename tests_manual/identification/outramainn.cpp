#include <motor.hpp>
#include <encoder.hpp>
#include <wifi.hpp>
#include <imu.hpp>

#define PWM_LEFT  100
#define PWM_RIGHT 100

typedef struct {
  uint32_t time_us;
  int16_t pwm_left;
  int16_t pwm_right;
  float w_left;
  float w_right;
  float w_imu;
} LogMessage;

uint8_t receiverAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // substitua com o MAC do receptor

LogMessage msg;

void setupe() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Motor::setup();
  Encoder::setup();
  IMU::setup();
}

void loope() {
  msg.time_us = micros();
  msg.pwm_left = PWM_LEFT;
  msg.pwm_right = PWM_RIGHT;
//   msg.w_left = Encoder::get_w_left();   // substitua por sua função real
//   msg.w_right = Encoder::get_w_right(); // substitua por sua função real
  msg.w_imu = IMU::get_w();             // substitua por sua função real

  Motor::move(MOTOR_LEFT, msg.pwm_left);
  Motor::move(MOTOR_RIGHT, msg.pwm_right);

  esp_now_send(receiverAddress, (uint8_t *)&msg, sizeof(msg));
  delay(10); // ~100Hz
}
