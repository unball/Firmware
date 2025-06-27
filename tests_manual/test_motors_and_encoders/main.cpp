// #include <motor.hpp>
// #include <encoder.hpp>

// #define TEST_DURATION_MS 2000
// #define MIN_SPEED_RAD_S 1.0

// void testMotorWithEncoder(uint8_t motorId, const char* name) {
//   Motor::move(motorId, 100);
//   delay(TEST_DURATION_MS);
//   Motor::stop();
//   double w = (motorId == MOTOR_LEFT ? Encoder::getMotorSpeeds().motorLeft : Encoder::getMotorSpeeds().motorRight);
//   Serial.printf("%s speed: %.2f rad/s\n", name, w);

//   if (abs(w) > MIN_SPEED_RAD_S) {
//     Serial.printf("[TEST RESULT] %s: PASS\n", name);
//   } else {
//     Serial.printf("[TEST RESULT] %s: FAIL\n", name);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("[TEST START] Motor + Encoder");

//   Motor::setup();
//   Encoder::setup();

// }

// void loop() {
//   delay(500);
//   testMotorWithEncoder(MOTOR_LEFT, "Motor A (LEFT)");
//   delay(500);
//   testMotorWithEncoder(MOTOR_RIGHT, "Motor B (RIGHT)");
// }
