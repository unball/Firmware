#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("[TEST START] ESP32 basic check");

  // If we got this far, the ESP32 is alive
  Serial.println("ESP32 is running and responded to upload.");

  Serial.println("[TEST RESULT] PASS");
}

void loop() {
  // Nothing to do
}
