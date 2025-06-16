#include <motor.hpp>
#include <encoder.hpp>
#include <waves.hpp>

#define LOG_INTERVAL_MS 10        // 10 ms = 100 Hz
#define NUM_SAMPLES 1000          // 10 s of data

typedef struct {
  uint32_t time_us;
  int16_t pwm;
  float w;
} LogMessage;

LogMessage msg;
unsigned long last_log_time = 0;
int sample_count = 0;

void step(int16_t *v1, int16_t *v2) {
  static uint32_t step_cont = 0;
  static int8_t step_flag = 0;

  if (step_cont > 200) step_flag = 1;  // Apply step after 2 seconds
  step_cont++;

  *v1 = 30 * step_flag;
  if (v2) *v2 = 30 * step_flag;  // Safety for dual-motor mode
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial monitor

  Motor::setup();
  Encoder::setup();

  Serial.println(F("time_us,pwm,w"));  // CSV header
}

void loop() {
  if (sample_count >= NUM_SAMPLES) {
    // Stop motor and halt program
    Motor::move(MOTOR_LEFT, 0);
    while (true);  // Infinite loop to stop
  }

  unsigned long now = millis();
  if (now - last_log_time >= LOG_INTERVAL_MS) {
    last_log_time = now;

    // Generate step input
    int16_t pwm;
    step(&pwm, nullptr);

    // Apply to motor
    Motor::move(MOTOR_LEFT, pwm);

    // Read encoder speed
    Encoder::vel vel = Encoder::getMotorSpeeds();
    float w_left = vel.motorLeft;

    // Log and print
    msg.time_us = micros();
    msg.pwm = pwm;
    msg.w = w_left;

    Serial.print(msg.time_us);
    Serial.print(',');
    Serial.print(msg.pwm);
    Serial.print(',');
    Serial.println(msg.w, 5);

    sample_count++;
  }
}
