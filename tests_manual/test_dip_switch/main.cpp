#include <Arduino.h>

#define DIP1_PIN 16
#define DIP2_PIN 17
#define DIP3_PIN 21

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(DIP1_PIN, INPUT_PULLUP);
  pinMode(DIP2_PIN, INPUT_PULLUP);
  pinMode(DIP3_PIN, INPUT_PULLUP);

  Serial.println("🎚️ DIP Switch test started.");
}

void loop() {
  Serial.print("DIP1: ");
  Serial.print(digitalRead(DIP1_PIN) == LOW ? "ON" : "OFF");
  Serial.print(" | DIP2: ");
  Serial.print(digitalRead(DIP2_PIN) == LOW ? "ON" : "OFF");
  Serial.print(" | DIP3: ");
  Serial.println(digitalRead(DIP3_PIN) == LOW ? "ON" : "OFF");
  delay(500);
}