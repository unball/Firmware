#include <Arduino.h>
#include "encoder.hpp"
#include "motor.hpp"
#include "config.h" // Assumo que as definições de pinos e R/v_max estão aqui
#include "control.hpp"

// === PARÂMETRO DE AJUSTE (SINTONIA) ===
// Comece com um valor MUITO baixo (ex: 1e-7 ou 0.000001) e aumente gradualmente
// até ver oscilação sustentada na velocidade máxima.
// Lembre-se: Estabilidade depende de gamma * u_c^2.
const double gamma_adapt = 0.00001; 

// === Parâmetros do Sistema ===
const double T = 0.01;               // Tempo de amostragem [s] (100Hz)
const double tau_m = 0.1;            // Constante de tempo do modelo (ajuste conforme desejado)
                                     // 0.1s é uma resposta rápida mas realista para motores DC comuns

// Coeficientes do Modelo de Referência (Discreto de 1ª Ordem)
const double am = exp(-T / tau_m);
const double bm = 1.0f - am;

// Variáveis de Estado
double omega = 0.0f;
double omega_m = 0.0f;
double u = 0.0f;
double r = 0.0f;

// Parâmetros Adaptativos (Inicializados com estimativa conservadora)
// Theta1 inicial baseada no ganho estático nominal
static double theta1 = 1.0; 
static double theta2 = 0.0; 

// Controle de Tempo
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    Encoder::setup();
    Motor::setup();
    
    // Inicialização segura
    theta1 = ((1023.0) * R) / v_max; // Estimativa inicial baseada na física
    theta2 = 0.0;
    
    delay(2000); // Tempo para preparar o Serial Plotter
}

// Função auxiliar de Deadzone do Motor (Hardware)
double applyMotorDeadzone(double u_in, double dz_val = 15.0f) {
    if (u_in > 0.0f) return (u_in > dz_val) ? u_in : dz_val;
    else if (u_in < 0.0f) return (u_in < -dz_val) ? u_in : -dz_val;
    return 0.0f;
}

void update_experiment() {
    // 1. Leitura da Velocidade Real
    Encoder::vel vel = Encoder::getMotorSpeeds();
    omega = vel.motorLeft; // Testando apenas um motor para sintonia

    // 2. Geração de Referência (Degrau de Pior Caso)
    // Após 2 segundos, aplica a velocidade MÁXIMA (69 rad/s)
    double t_sec = millis() / 1000.0;
    if (t_sec > 2.0) {
        r = 69.0; 
    } else {
        r = 0.0;
    }

    // 3. Modelo de Referência
    omega_m = am * omega_m + bm * r;

    // 4. Cálculo do Erro de Rastreamento
    double e = omega - omega_m;

    // 5. Regra MIT Pura (Sem robustez)
    // dTheta1/dt = -gamma * r * e
    // dTheta2/dt =  gamma * y * e
    // Nota: Sinais baseados na definição e = y - ym ou ym - y. 
    // Mantendo consistência com seu código anterior (e = omega - omega_m):
    
    double delta_theta1 = -T * gamma_adapt * r * e;
    double delta_theta2 =  T * gamma_adapt * omega * e;

    // Atualização dos parâmetros (sem projeção/limites para ver a deriva real)
    theta1 += delta_theta1;
    theta2 += delta_theta2;

    // 6. Lei de Controle
    double u_unsat = theta1 * r - theta2 * omega;

    // 7. Saturação do Atuador (1023 PWM)
    // Importante: A saturação física existe, então mantemos no código
    u = constrain(u_unsat, -1023.0f, 1023.0f);

    // Aplica no motor
    double u_final = applyMotorDeadzone(u, 73.0); // Usando seu deadzone de 73
    Motor::move(MOTOR_LEFT, u_final);

    // 8. Telemetria para Serial Plotter
    // Formato: "label:valor"
    Serial.print("Ref:"); Serial.print(r);
    Serial.print(",Modelo:"); Serial.print(omega_m);
    Serial.print(",Real:"); Serial.print(omega);
    Serial.print(",PWM:"); Serial.print(u / 10.0); // Escalado para caber no gráfico
    Serial.print(",Theta1:"); Serial.print(theta1);
    Serial.print(",Theta2:"); Serial.println(theta2);
}

void loop() {
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(T * 1000)) {
        last_time = now;
        update_experiment();
    }
}