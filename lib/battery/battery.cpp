#include <Arduino.h>
#include "battery.hpp"

#define BATTERY_PIN 22               // ADC input pin
#define ADC_REF_VOLTAGE 3.3f         // ESP32 ADC reference voltage
#define ADC_RESOLUTION 4095.0f       // 12-bit ADC
#define BATTERY_SCALE 2.424f         // (R1 + R2) / R2 = 80k / 33k

namespace Battery {
    void setup() {
        pinMode(BATTERY_PIN, INPUT);
    }

    float readVoltage() {
        int raw = analogRead(BATTERY_PIN);
        float voltage = (raw * ADC_REF_VOLTAGE / ADC_RESOLUTION) * BATTERY_SCALE;
        return voltage;
    }
}
