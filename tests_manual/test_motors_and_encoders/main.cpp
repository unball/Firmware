#include <motor.hpp>
#include <encoder.hpp>

#define TEST_DURATION_MS 2000
#define MIN_SPEED_RAD_S 1.0

void testMotorWithEncoder(uint8_t motorId, const char* name) {
  Motor::move(motorId, 200);  // Positive power for forward motion
  delay(TEST_DURATION_MS);
  Motor::stop();

  double w = Encoder::getMotorSpeeds();
  Serial.printf("%s speed: %.2f rad/s\n", name, w);

  if (w > MIN_SPEED_RAD_S) {
    Serial.printf("[TEST RESULT] %s: PASS\n", name);
  } else {
    Serial.printf("[TEST RESULT] %s: FAIL\n", name);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[TEST START] Motor + Encoder (using user libs)");

  Motor::setup();
  Encoder::setup();

  delay(500);

  testMotorWithEncoder(MOTOR_LEFT, "Motor A");
  delay(1000);
  testMotorWithEncoder(MOTOR_RIGHT, "Motor B");
}

void loop() {
  // No loop needed
}
