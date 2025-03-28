#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("✅ ESP32 is alive and communicating over Serial!");
}

void loop() {
  // Nothing to do
}