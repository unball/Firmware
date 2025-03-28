#include <Arduino.h>
#include "battery.hpp"

#define BAT_MIN_VOLTAGE 5.0f
#define BAT_MAX_VOLTAGE 7.85f
#define NUM_SAMPLES 5
#define SAMPLE_DELAY_MS 100
#define MAX_STD_DEV 0.1f  // Maximum allowed noise

float calcStdDev(float* values, int count, float mean) {
  float sumSq = 0;
  for (int i = 0; i < count; i++) {
    sumSq += pow(values[i] - mean, 2);
  }
  return sqrt(sumSq / count);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[TEST START] Battery voltage (with stability check)");

  Battery::setup();

  float readings[NUM_SAMPLES];
  float sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    readings[i] = Battery::readVoltage();
    sum += readings[i];
    delay(SAMPLE_DELAY_MS);
  }

  float avg = sum / NUM_SAMPLES;
  float stddev = calcStdDev(readings, NUM_SAMPLES, avg);

  Serial.printf("Voltage readings (V): ");
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.printf("%.2f ", readings[i]);
  }
  Serial.println();
  Serial.printf("Average: %.2f V | Std Dev: %.3f V\n", avg, stddev);

  bool inRange = (avg >= BAT_MIN_VOLTAGE && avg <= BAT_MAX_VOLTAGE);
  bool stable = (stddev <= MAX_STD_DEV);

  if (inRange && stable) {
    Serial.println("[TEST RESULT] PASS");
  } else {
    if (!inRange) Serial.println("⚠️ Voltage out of expected range.");
    if (!stable) Serial.println("⚠️ Voltage reading is unstable.");
    Serial.println("[TEST RESULT] FAIL");
  }
}

void loop() {
  // Nothing to do
}
