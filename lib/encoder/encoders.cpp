// #include "encoder.hpp"
// #include "Arduino.h" // Para portENTER_CRITICAL_ISR, etc.

// // --- Constantes ---
// // Defina aqui os pinos do seu hardware
// // const int ENC_MOTOR_A_CHA_PIN = 34;
// // const int ENC_MOTOR_A_CHB_PIN = 35;
// // const int ENC_MOTOR_B_CHA_PIN = 32;
// // const int ENC_MOTOR_B_CHB_PIN = 33;

// // Propriedades do Encoder e Motor
// // Exemplo: Encoder de 12 CPR (Counts Per Revolution) na quadratura 4x -> 48 pulsos por volta do encoder.
// // Se houver uma caixa de redução 30:1, seria 48 * 30 = 1440 pulsos por volta do eixo.
// // Ajuste este valor para o seu hardware!
// constexpr float PULSES_PER_REVOLUTION = 48.0; 
// constexpr float RADS_PER_PULSE = (2.0 * PI) / PULSES_PER_REVOLUTION;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// namespace Encoder {

//     // Contadores de pulso. Volatile para garantir que o compilador não otimize o acesso.
//     volatile int32_t pulse_count_A = 0;
//     volatile int32_t pulse_count_B = 0;

//     // Variáveis para o cálculo da velocidade no loop principal
//     int32_t last_pulse_count_A = 0;
//     int32_t last_pulse_count_B = 0;
//     unsigned long last_update_time_us = 0;

//     // Velocidades calculadas (rad/s)
//     float current_speed_A = 0.0;
//     float current_speed_B = 0.0;

//     // --- Interrupt Service Routines (ISRs) ---
//     // ISRs devem ser o mais curtas e rápidas possível.
    
//     // ISR para o motor A
//     void IRAM_ATTR isr_motor_A() {
//         portENTER_CRITICAL_ISR(&timerMux); // Protege o acesso ao contador
//         // Lógica de quadratura:
//         // Se o canal B está diferente do canal A, está girando em uma direção.
//         // Se estiverem iguais, na outra direção.
//         if (digitalRead(ENC_MOTOR_A_CHB_PIN) != digitalRead(ENC_MOTOR_A_CHA_PIN)) {
//             pulse_count_A++;
//         } else {
//             pulse_count_A--;
//         }
//         portEXIT_CRITICAL_ISR(&timerMux);
//     }

//     // ISR para o motor B
//     void IRAM_ATTR isr_motor_B() {
//         portENTER_CRITICAL_ISR(&timerMux); // Protege o acesso ao contador
//         if (digitalRead(ENC_MOTOR_B_CHB_PIN) != digitalRead(ENC_MOTOR_B_CHA_PIN)) {
//             pulse_count_B++;
//         } else {
//             pulse_count_B--;
//         }
//         portEXIT_CRITICAL_ISR(&timerMux);
//     }
    
//     // --- Funções Principais ---

//     void setup() {
//         // Zera as variáveis
//         pulse_count_A = 0;
//         pulse_count_B = 0;
//         last_pulse_count_A = 0;
//         last_pulse_count_B = 0;
//         current_speed_A = 0.0;
//         current_speed_B = 0.0;
//         last_update_time_us = micros();

//         // Configura os pinos
//         pinMode(ENC_MOTOR_A_CHA_PIN, INPUT_PULLUP);
//         pinMode(ENC_MOTOR_A_CHB_PIN, INPUT_PULLUP);
//         pinMode(ENC_MOTOR_B_CHA_PIN, INPUT_PULLUP);
//         pinMode(ENC_MOTOR_B_CHB_PIN, INPUT_PULLUP);

//         // Anexa as interrupções. CHANGE detecta tanto subida quanto descida (melhora a resolução em 4x).
//         attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_A_CHA_PIN), isr_motor_A, CHANGE);
//         attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_B_CHA_PIN), isr_motor_B, CHANGE);
//     }

//     // Esta função deve ser chamada periodicamente pelo loop principal
//     void updateSpeeds() {
//         unsigned long now_us = micros();
//         long delta_t_us = now_us - last_update_time_us;

//         // Copia os contadores de pulso de forma segura (atômica)
//         int32_t current_pulses_A;
//         int32_t current_pulses_B;

//         portENTER_CRITICAL(&timerMux);
//         current_pulses_A = pulse_count_A;
//         current_pulses_B = pulse_count_B;
//         portEXIT_CRITICAL(&timerMux);
        
//         // Calcula a variação de pulsos desde a última chamada
//         long delta_pulses_A = current_pulses_A - last_pulse_count_A;
//         long delta_pulses_B = current_pulses_B - last_pulse_count_B;

//         // Atualiza as variáveis para a próxima iteração
//         last_pulse_count_A = current_pulses_A;
//         last_pulse_count_B = current_pulses_B;
//         last_update_time_us = now_us;

//         // Calcula a velocidade (rad/s) - todo o ponto flutuante é feito aqui
//         // velocidade = (delta_pulsos * rads_por_pulso) / delta_t_em_segundos
//         if (delta_t_us > 0) {
//             current_speed_A = (float)(delta_pulses_A * RADS_PER_PULSE) / (delta_t_us * 1e-6);
//             current_speed_B = (float)(delta_pulses_B * RADS_PER_PULSE) / (delta_t_us * 1e-6);
//         } else {
//             current_speed_A = 0;
//             current_speed_B = 0;
//         }
//     }

//     vel getMotorSpeeds() {
//         vel result;
//         // Simplesmente retorna as últimas velocidades calculadas por updateSpeeds()
//         result.motorLeft = current_speed_A;
//         result.motorRight = current_speed_B;
//         return result;
//     }
// }

// // #include "encoder.hpp"

// // #define alpha 0.95
// // #define beta 0.05

// // const int watchdog_timer = 500000; // us
// // constexpr uint64_t MIN_VALID_T_US = 500;    // Max RPM = 650 -> T_us = 7700us

// // namespace Encoder {

// //     hw_timer_t *Timer0_Cfg = NULL;
// //     hw_timer_t *Timer1_Cfg = NULL;

// //     volatile float present_speed_A;
// //     volatile float present_speed_B;

// //     void IRAM_ATTR Ext_INT1_ISR() {
// //         detachInterrupt(ENC_MOTOR_A_CHA_PIN);
        
// //         uint64_t T_us = timerReadMicros(Timer0_Cfg);

// //         int direction = digitalRead(ENC_MOTOR_A_CHB_PIN);
// //         direction = direction == 1? 1:-1;

// //         // Filters out unrealistically short pulses caused by noise or startup
// //         if (T_us >= MIN_VALID_T_US) {
// //             present_speed_A = (direction)*(2*PI)/(12*(T_us*1e-6));
// //         }

// //         timerRestart(Timer0_Cfg);

// //         attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
// //     }

// //     void IRAM_ATTR Ext_INT2_ISR() {
// //         detachInterrupt(ENC_MOTOR_B_CHA_PIN);
        
// //         uint64_t T_us = timerReadMicros(Timer1_Cfg);

// //         int direction = digitalRead(ENC_MOTOR_B_CHB_PIN);
// //         direction = direction == 1? 1:-1;
        
// //         // Filters out unrealistically short pulses caused by noise or startup.
// //         if (T_us >= MIN_VALID_T_US) {
// //             present_speed_B = (direction)*(2*PI)/(12*(T_us*1e-6));
// //         }

// //         timerRestart(Timer1_Cfg);

// //         attachInterrupt(ENC_MOTOR_B_CHA_PIN, Ext_INT2_ISR, RISING);
// //     }

// //     void setup() {
// //         present_speed_A = 0;
// //         present_speed_B = 0;

// //         // Motor A
// //         pinMode(ENC_MOTOR_A_CHA_PIN, INPUT);
// //         pinMode(ENC_MOTOR_A_CHB_PIN, INPUT);
// //         attachInterrupt(ENC_MOTOR_A_CHA_PIN, Ext_INT1_ISR, RISING);
// //         // Configure Timer0
// //         Timer0_Cfg = timerBegin(0, 2, true);
// //         timerWrite(Timer0_Cfg, 0);
// //         timerStart(Timer0_Cfg);

// //         // Motor B
// //         pinMode(ENC_MOTOR_B_CHA_PIN, INPUT);
// //         pinMode(ENC_MOTOR_B_CHB_PIN, INPUT);
// //         attachInterrupt(ENC_MOTOR_B_CHA_PIN, Ext_INT2_ISR, RISING);
// //         // Configure Timer0
// //         Timer1_Cfg = timerBegin(1, 2, true);
// //         timerWrite(Timer1_Cfg, 0);
// //         timerStart(Timer1_Cfg);
// //     }

// //     vel getMotorSpeeds() {
// //         vel result;

// //         if (timerReadMicros(Timer0_Cfg) > watchdog_timer) {
// //             present_speed_A = 0;
// //         }

// //         if (timerReadMicros(Timer1_Cfg) > watchdog_timer) {
// //             present_speed_B = 0;
// //         }

// //         result.motorLeft = present_speed_A;
// //         result.motorRight = present_speed_B;

// //         return result;
// //     }
// // }