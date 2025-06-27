// #include <motor.hpp>
// #include <encoder.hpp>

// #define LOG_INTERVAL_US 500        // 500 µs = 2 kHz
// #define NUM_SAMPLES     5000       // 2.5 seconds of data
// #define STEP_DELAY_MS   1000       // Apply step after 1 second

// struct LogData {
//   uint32_t time_us;
//   int16_t pwm;
//   float w;
// };

// LogData logBuffer[NUM_SAMPLES];
// uint16_t sample_count = 0;
// uint32_t start_time_us;
// uint32_t next_sample_us;
// uint16_t missed_count = 0;

// bool step_applied = false;
// int16_t pwm_value = 0;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);  // Wait for serial monitor

//   Motor::setup();
//   Encoder::setup();

//   start_time_us = micros();
//   next_sample_us = start_time_us;
// }

// void loop() {
//   uint32_t now = micros();
//   uint32_t elapsed_time_ms = (now - start_time_us) / 1000;

//   // Apply step input once after 1 second
//   if (!step_applied && elapsed_time_ms >= STEP_DELAY_MS) {
//     pwm_value = 30;
//     // Motor::move(MOTOR_LEFT, pwm_value);
//     Motor::move(MOTOR_RIGHT, pwm_value);
//     step_applied = true;
//   }

//   // Sampling and logging
//   if (sample_count < NUM_SAMPLES && now >= next_sample_us) {
//     if (now > next_sample_us + LOG_INTERVAL_US) {
//       missed_count++;
//     }

//     Encoder::vel vel = Encoder::getMotorSpeeds();

//     logBuffer[sample_count].time_us = now - start_time_us;
//     logBuffer[sample_count].pwm = pwm_value;
//     logBuffer[sample_count].w = vel.motorRight; + vel.motorLeft;

//     sample_count++;
//     next_sample_us += LOG_INTERVAL_US;
//   }

//   // Done collecting
//   if (sample_count == NUM_SAMPLES) {
//     // Motor::move(MOTOR_LEFT, 0);  // stop motor
//     Motor::move(MOTOR_RIGHT, 0);  // stop motor
//     Serial.println("time_us,pwm,w");

//     for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
//       float time_s = logBuffer[i].time_us / 1e6;
//       Serial.print(time_s, 5);
//       Serial.print(',');
//       Serial.print(logBuffer[i].pwm);
//       Serial.print(',');
//       Serial.println(logBuffer[i].w, 5);
//     }

//     Serial.print("Missed samples: ");
//     Serial.println(missed_count);
//     while (true);  // halt
//   }
// }
