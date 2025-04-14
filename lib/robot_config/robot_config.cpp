#include "robot_config.hpp"
#include <Arduino.h>

namespace {
    constexpr int PIN_DIP1 = 21;  // LSB
    constexpr int PIN_DIP2 = 16;
    constexpr int PIN_DIP3 = 17;  // MSB

    RobotConfig::Config config;

    int readDIP() {
        int b0 = digitalRead(PIN_DIP1);
        int b1 = digitalRead(PIN_DIP2);
        int b2 = digitalRead(PIN_DIP3);
        return (b2 << 2) | (b1 << 1) | b0;
    }

    RobotConfig::Config decodeConfig(int dipValue) {
        using namespace RobotConfig;

        switch (dipValue) {
            case 0: return {0, Mode::NORMAL};
            case 1: return {1, Mode::NORMAL};
            case 2: return {2, Mode::NORMAL};
            case 3: return {0, Mode::CONTROL_TESTER};
            case 4: return {0, Mode::DEAD_ZONE_TESTER};
            case 5: return {0, Mode::TWIDDLE};
            default: return {0, Mode::NORMAL}; // fallback
        }
    }
}

namespace RobotConfig {

    void setup() {
        pinMode(PIN_DIP1, INPUT_PULLUP);
        pinMode(PIN_DIP2, INPUT_PULLUP);
        pinMode(PIN_DIP3, INPUT_PULLUP);
        int dipValue = readDIP();
        config = decodeConfig(dipValue);

        Serial.printf("[CONFIG] Robot: %d | Mode: %d\n", config.robotNumber, static_cast<int>(config.mode));
    }

    const Config& get() {
        return config;
    }

    int getRobotNumber() {
        return config.robotNumber;
    }

    Mode getMode() {
        return config.mode;
    }

    bool isTwiddle() {
        return config.mode == Mode::TWIDDLE;
    }

    bool isControlTester() {
        return config.mode == Mode::CONTROL_TESTER;
    }

    bool isDeadZoneTester() {
        return config.mode == Mode::DEAD_ZONE_TESTER;
    }
}
