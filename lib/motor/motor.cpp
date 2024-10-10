#include "motor.hpp"

namespace Motor {

    int8_t motor_direction[2] = {0, 0};

    uint8_t pin1[2] = {AIN1_PIN, BIN1_PIN};
    uint8_t pin2[2] = {AIN2_PIN, BIN2_PIN};
    uint8_t PWM[2] = {PWMA_PIN, PWMB_PIN};

    const int freq = 5000;        // Frequência PWM
    const int resolution = 20;    // Resolução de 20 bits
    const int maxPWM = (1 << resolution) - 1;  // 2^20 - 1 = 1048575
    const int pwmChannelA = 0;    // Canal PWM para motor A
    const int pwmChannelB = 1;    // Canal PWM para motor B

    int8_t getMotorDirection(int8_t motor) {
        return motor_direction[motor];
    }

    void setup(void) {
        pinMode(AIN1_PIN, OUTPUT);
        pinMode(AIN2_PIN, OUTPUT);

        pinMode(STBY_PIN, OUTPUT);

        pinMode(BIN1_PIN, OUTPUT);
        pinMode(BIN2_PIN, OUTPUT);

        // Configura os canais PWM com a maior resolução (20 bits)
        ledcSetup(pwmChannelA, freq, resolution);
        ledcSetup(pwmChannelB, freq, resolution);

        // Anexa os pinos PWM aos canais correspondentes
        ledcAttachPin(PWMA_PIN, pwmChannelA);
        ledcAttachPin(PWMB_PIN, pwmChannelB);
    }

    void move(uint8_t motor, int32_t power) {
        // Permite corrente para os motores
        digitalWrite(STBY_PIN, HIGH);

        // Saturador
        power = power > 255 ? 255 : power;
        power = power < -255 ? -255 : power;

        // Converte o valor de 0-255 para a escala de 20 bits (0-1048575)
        int pwmValue = map(abs(power), 0, 255, 0, maxPWM);

        // Define a direção do motor
        motor_direction[motor] = (power > 0 ? 1 : -1);
        int inPin1;
        int inPin2;
        if (motor_direction[motor] == -1) {
            inPin1 = HIGH;
            inPin2 = LOW;
        }
        else {
            inPin1 = LOW;
            inPin2 = HIGH;
        }

        // Escreve a direção e potência do motor
        digitalWrite(pin1[motor], inPin1);
        digitalWrite(pin2[motor], inPin2);

        // Usa o ledcWrite para aplicar PWM
        if (motor == 0) {
            ledcWrite(pwmChannelA, pwmValue);
        } else {
            ledcWrite(pwmChannelB, pwmValue);
        }
    }

    // Para os motores
    void stop() {
        digitalWrite(STBY_PIN, LOW);
    }
}
