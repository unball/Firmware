#include <Arduino.h>

#define MBAT_PIN 22
#define VOLTAGE_DIVIDER_RATIO 2.0

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔋 Battery voltage reading started.");

  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(MBAT_PIN);
  float voltage = (raw / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;

  Serial.print("Battery voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  delay(1000);
}