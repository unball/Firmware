#include <Arduino.h>

#define MOTOR_A_PWM 32
#define MOTOR_A_IN1 25
#define MOTOR_A_IN2 33

#define MOTOR_B_PWM 12
#define MOTOR_B_IN1 27
#define MOTOR_B_IN2 14

#define STBY_PIN 26

void motorTest(int in1, int in2, int pwm, const char* label) {
  Serial.print("Testing ");
  Serial.println(label);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 200);
  delay(1500);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 0);
  delay(500);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(pwm, 200);
  delay(1500);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚦 Starting Motor Test");

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);

  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
}

void loop() {
  motorTest(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, "Motor A");
  motorTest(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, "Motor B");
}