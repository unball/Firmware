#include <motor.hpp>
#include <encoder.hpp>

#define LOG_INTERVAL_US 2000     // 500 Hz = 2000 µs
#define NUM_SAMPLES     1000     // Adjust based on memory available

struct LogData {
  uint32_t time_us;
  int16_t pwm;
  float w;
};

LogData logBuffer[NUM_SAMPLES];
uint16_t sample_count = 0;

uint32_t start_time_us;
uint32_t next_sample_us;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Motor::setup();
  Encoder::setup();

  // Step input applied immediately (you can modify if needed)
  Motor::move(MOTOR_LEFT, 30);

  start_time_us = micros();
  next_sample_us = start_time_us;
}

void loop() {
  uint32_t now = micros();
  if (sample_count < NUM_SAMPLES && now >= next_sample_us) {
    // Read speed
    Encoder::vel vel = Encoder::getMotorSpeeds();

    // Log data
    logBuffer[sample_count].time_us = now - start_time_us;
    logBuffer[sample_count].pwm = 30;
    logBuffer[sample_count].w = vel.motorLeft;

    sample_count++;
    next_sample_us += LOG_INTERVAL_US;
  }

  // Done collecting: stop motor and print
  if (sample_count == NUM_SAMPLES) {
    Motor::move(MOTOR_LEFT, 0);
    Serial.println("time_us,pwm,w");

    for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
      Serial.print(logBuffer[i].time_us);
      Serial.print(',');
      Serial.print(logBuffer[i].pwm);
      Serial.print(',');
      Serial.println(logBuffer[i].w, 5);
    }

    while (true);  // halt
  }
}
