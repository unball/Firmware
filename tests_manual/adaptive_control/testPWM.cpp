#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h"

#define PWM_START     0
#define PWM_END       1023
#define PWM_STEP      25
#define SETTLE_TIME   1000  // tempo para estabilizar (ms)
#define SAMPLE_COUNT  20    // número de leituras para média
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
        while (true);                // fim do experimento
    }
}
