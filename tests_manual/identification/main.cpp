#include <motor.hpp>
#include <encoder.hpp>
#include <wifi.hpp>
#include <imu.hpp>
#include <waves.hpp>
#include <motor.hpp>
#include <encoder.hpp>
#include <wifi.hpp>
#include <imu.hpp>

#define PWM_STEP  100  // valor do degrau

typedef struct {
  uint32_t time_us;
  int16_t pwm_left;
  int16_t pwm_right;
  float w_left;
  float w_right;
  float w_imu;
} LogMessage;

uint8_t receiverAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // MAC do receptor

LogMessage msg;

// === Geração do degrau ===
// void step(int16_t *v1, int16_t *v2) {
//   static uint32_t step_cont = 0;
//   static int8_t step_flag = 0;

//   if (step_cont > 200) step_flag = 1;
//   if (step_cont > 2000) step_flag = 0;

//   step_cont++;
//   *v1 = PWM_STEP * step_flag;
//   *v2 = PWM_STEP * step_flag;
// }

void setup() {
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
  Serial.println(F("Setup done!"));
}

void loop() {
  int16_t pwmL, pwmR;
  Waves::step(&pwmL, &pwmR);
//   step(&pwmL, &pwmR);

  msg.time_us = micros();
  msg.pwm_left = pwmL;
  msg.pwm_right = pwmR;
  Encoder::vel vel = Encoder::getMotorSpeeds();
  msg.w_left = vel.motorLeft;
  msg.w_right = vel.motorRight;
  msg.w_imu = IMU::get_w();

  Motor::move(MOTOR_LEFT, pwmL);
  Motor::move(MOTOR_RIGHT, pwmR);
  Serial.print(F("pwms:"));
  Serial.print(pwmL);
  Serial.print(F(","));
  Serial.println(pwmR);


  esp_now_send(receiverAddress, (uint8_t *)&msg, sizeof(msg));
  delay(10); // 100 Hz
}
