// robot_config.hpp
#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

namespace RobotConfig {
    enum class Mode {
        GAME,
        CONTROL_TESTER,
        DEAD_ZONE_TESTER,
        TWIDDLE,
        DEBUG
    };

    struct Config {
        int robotNumber;
        Mode mode;
    };

    void setup();                 // Chamada no setup()
    const Config& get();         // Acesso à config lida do DIP
    const char* modeToString();  // Returns the readable name of the current mode

    // Atalhos práticos:
    int getRobotNumber();
    Mode getMode();

    // Bools para lógica condicional
    bool isDebug();
    bool isTwiddle();
    bool isControlTester();
    bool isDeadZoneTester();
}

#endif
