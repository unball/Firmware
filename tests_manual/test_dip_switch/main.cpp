#include <Arduino.h>

#define DIP1_PIN 16
#define DIP2_PIN 17
#define DIP3_PIN 21

int readStablePin(int pin) {
  int first = digitalRead(pin);
  delay(10);  // Small delay to detect instability
  int second = digitalRead(pin);
  return (first == second) ? first : -1;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[TEST START] DIP Switch");

  pinMode(DIP1_PIN, INPUT_PULLUP);
  pinMode(DIP2_PIN, INPUT_PULLUP);
  pinMode(DIP3_PIN, INPUT_PULLUP);

  int d1 = readStablePin(DIP1_PIN);
  int d2 = readStablePin(DIP2_PIN);
  int d3 = readStablePin(DIP3_PIN);

  bool stable = (d1 != -1 && d2 != -1 && d3 != -1);

  if (!stable) {
    Serial.println("⚠️ Unstable or floating pin detected.");
    if (d1 == -1) Serial.println("  → DIP 1 is unstable");
    if (d2 == -1) Serial.println("  → DIP 2 is unstable");
    if (d3 == -1) Serial.println("  → DIP 3 is unstable");
    Serial.println("[TEST RESULT] FAIL");
    return;
  }

  int dipValue = (d1 << 2) | (d2 << 1) | d3;

  Serial.printf("DIP Switch value: %d (binary: %03b)\n", dipValue, dipValue);
  Serial.printf("  DIP 1: %s\n", d1 ? "ON (HIGH)" : "OFF (LOW)");
  Serial.printf("  DIP 2: %s\n", d2 ? "ON (HIGH)" : "OFF (LOW)");
  Serial.printf("  DIP 3: %s\n", d3 ? "ON (HIGH)" : "OFF (LOW)");

  if (dipValue >= 0 && dipValue <= 7) {
    Serial.println("[TEST RESULT] PASS");
  } else {
    Serial.println("[TEST RESULT] FAIL");
  }
}

void loop() {
  // Nothing here
}
