#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"

#define PWM_START     0
#define PWM_END       1023
#define PWM_STEP      1
#define SETTLE_TIME   10  // tempo para estabilizar (ms)
#define SAMPLE_COUNT  10    // número de leituras para média
#define SAMPLE_DELAY  10    // intervalo entre leituras (ms)

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    delay(1000);  // espera inicial

    Serial.println("pwm,omega");  // header do CSV
}

void loop() {
    static int pwm_value = PWM_START;

    Motor::move(MOTOR_LEFT, pwm_value);  // aplica PWM fixo
    Motor::move(MOTOR_RIGHT, pwm_value);  // aplica PWM fixo
    delay(SETTLE_TIME);  // espera estabilizar

    // Coleta e calcula média de omega
    float omega_sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        Encoder::vel vel = Encoder::getMotorSpeeds();
        omega_sum += vel.motorLeft;
        delay(SAMPLE_DELAY);
    }

    float omega_avg = omega_sum / SAMPLE_COUNT;

    // Envia resultado via Serial
    Serial.print(pwm_value);
    Serial.print(",");
    Serial.println(omega_avg, 4);

    pwm_value += PWM_STEP;
    if (pwm_value > PWM_END) {
        Motor::move(MOTOR_LEFT, 0);  // desliga motor
        Motor::move(MOTOR_RIGHT, 0);  // desliga motor
        while (true);                // fim do experimento
    }
}



// 1015,82.7057
// 1016,82.7874
// 1017,82.3691
// 1018,83.3545
// 1019,83.0626
// 1020,83.1811
// 1021,82.9279
// 1022,83.2387
// 1023,83.4769