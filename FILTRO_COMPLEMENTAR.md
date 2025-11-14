# Filtro Complementar para Velocidade Angular

## Visão Geral

Implementação de um **filtro complementar** para fusão de dados de velocidade angular do robô, combinando:
- **IMU (Giroscópio)**: Leituras diretas de velocidade angular (ω)
- **Encoders**: Velocidade angular calculada a partir das velocidades das rodas

## Teoria

### Cinemática do Robô Diferencial

Para um robô com tração diferencial:

```
ω = (v_direita - v_esquerda) / L
```

Onde:
- `ω` = velocidade angular do robô [rad/s]
- `v_direita` = velocidade linear da roda direita [m/s]
- `v_esquerda` = velocidade linear da roda esquerda [m/s]
- `L` = distância entre as rodas (wheelbase) [m] = **0.0825 m**

A velocidade linear de cada roda é:
```
v = ω_roda × R
```
Onde `R` = raio da roda = **0.0215 m**

### Filtro Complementar

O filtro complementar combina as duas fontes de dados:

```
ω_filtrado = α × ω_giroscópio + (1 - α) × ω_encoders
```

**Parâmetros:**
- `α` = peso do giroscópio (tipicamente 0.95 - 0.98)
- `1 - α` = peso dos encoders

### Por que usar filtro complementar?

| Sensor | Vantagens | Desvantagens |
|--------|-----------|--------------|
| **Giroscópio (IMU)** | • Resposta rápida<br>• Mede mudanças instantâneas<br>• Alta frequência | • Deriva ao longo do tempo<br>• Acumula erros<br>• Sensível a ruído |
| **Encoders** | • Precisão a longo prazo<br>• Sem deriva<br>• Baseado em medição física | • Atraso na resposta<br>• Baixa resolução em baixas velocidades<br>• Pode ser zero quando parado |
| **Filtro Complementar** | • Melhor dos dois mundos<br>• Resposta rápida sem deriva<br>• Robusto | • Requer ajuste do parâmetro α |

## Implementação

### Arquivos Modificados

1. **`lib/imu/imu.hpp`** e **`lib/imu/imu.cpp`**
   - Adicionada função `get_w_filtered(float w_encoders, float alpha = 0.98)`
   
2. **`lib/encoder/encoder.hpp`** e **`lib/encoder/encoder.cpp`**
   - Adicionada função `getAngularVelocity(float R, float L)`

### Funções Criadas

#### `IMU::get_w_filtered()`
```cpp
float IMU::get_w_filtered(float w_encoders, float alpha = 0.98)
```
- **Entrada**: velocidade angular dos encoders, coeficiente alpha
- **Saída**: velocidade angular filtrada [rad/s]
- **Uso**: Chame esta função no loop de controle

#### `Encoder::getAngularVelocity()`
```cpp
float Encoder::getAngularVelocity(float R = 0.0215f, float L = 0.0825f)
```
- **Entrada**: raio da roda (R), distância entre rodas (L)
- **Saída**: velocidade angular calculada dos encoders [rad/s]
- **Uso**: Calcula ω a partir das velocidades das rodas

## Como Usar

### Exemplo Básico

```cpp
#include "encoder.hpp"
#include "imu.hpp"

void loop() {
    // 1. Calcular velocidade angular dos encoders
    float w_encoders = Encoder::getAngularVelocity();
    
    // 2. Aplicar filtro complementar
    float w_filtered = IMU::get_w_filtered(w_encoders, 0.98);
    
    // 3. Usar w_filtered no controle
    // ... seu código de controle aqui
}
```

### Integração no Controlador PID

```cpp
// No loop de controle
float w_encoders = Encoder::getAngularVelocity();
float w = IMU::get_w_filtered(w_encoders, 0.98);

// Usar w ao invés da leitura crua do giroscópio
PIDController::update(v_ref, w);
```

### Integração no Controlador Adaptativo

```cpp
// Dentro do loop de controle adaptativo
float w_encoders = Encoder::getAngularVelocity(R, L);
float w_filtered = IMU::get_w_filtered(w_encoders, 0.98);

// Usar w_filtered para realimentação
// ... atualizar parâmetros adaptativos
```

## Ajuste do Parâmetro α (Alpha)

### Valores Recomendados

| Situação | α recomendado | Justificativa |
|----------|---------------|---------------|
| **Padrão/Geral** | 0.98 | Bom equilíbrio para maioria dos casos |
| **Movimentos rápidos** | 0.99 | Prioriza resposta rápida do giroscópio |
| **Movimento lento/preciso** | 0.95 | Mais confiança nos encoders |
| **Operação longa** | 0.90-0.95 | Evita deriva acumulada |

### Como Ajustar

1. **Teste com α = 0.98** (valor padrão)
2. **Se houver deriva**: diminua α (tente 0.95, depois 0.90)
3. **Se resposta estiver lenta**: aumente α (tente 0.99)
4. **Para otimizar**: capture dados e plote as três curvas:
   - `w_gyro` (linha pontilhada)
   - `w_encoders` (linha tracejada)
   - `w_filtered` (linha sólida)

### Alpha Adaptativo (Avançado)

Você pode variar α dinamicamente baseado na situação:

```cpp
float w_encoders = Encoder::getAngularVelocity();

// Usar mais giroscópio em manobras rápidas
float alpha;
if (abs(w_ref) > 3.0) {
    alpha = 0.99;  // Movimento rápido
} else {
    alpha = 0.95;  // Movimento lento
}

float w = IMU::get_w_filtered(w_encoders, alpha);
```

## Exemplo de Teste

Um arquivo de exemplo completo está disponível em:
**`tests_manual/complementary_filter_example.cpp`**

Este exemplo:
- Imprime as três leituras (giroscópio, encoders, filtrado)
- Formato CSV para análise
- Taxa de amostragem de 100 Hz

### Como executar o teste:

1. Copie o conteúdo de `complementary_filter_example.cpp` para `src/main.cpp`
2. Compile e carregue no ESP32
3. Abra o Serial Monitor (115200 baud)
4. Mova o robô manualmente ou via controle
5. Copie os dados e plote em Python/Excel

## Análise de Dados

Para visualizar os resultados, use este script Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Ler dados do Serial Monitor (copie e salve como csv)
df = pd.read_csv('complementary_filter_data.csv', 
                 names=['time', 'w_gyro', 'w_encoders', 'w_filtered'])

plt.figure(figsize=(12, 6))
plt.plot(df['time'], df['w_gyro'], 'r--', alpha=0.5, label='Giroscópio (IMU)')
plt.plot(df['time'], df['w_encoders'], 'b:', alpha=0.5, label='Encoders')
plt.plot(df['time'], df['w_filtered'], 'g-', linewidth=2, label='Filtrado')
plt.xlabel('Tempo (ms)')
plt.ylabel('Velocidade Angular (rad/s)')
plt.legend()
plt.grid(True)
plt.title('Comparação: Giroscópio vs Encoders vs Filtro Complementar')
plt.show()
```

## Solução de Problemas

### Problema: Leitura filtrada deriva ao longo do tempo
**Solução**: Diminua α (tente 0.95 ou 0.90)

### Problema: Resposta muito lenta nas curvas
**Solução**: Aumente α (tente 0.99)

### Problema: Oscilações na leitura filtrada
**Causas possíveis**:
- Ruído nos encoders
- α muito baixo
**Solução**: 
- Verifique conexões dos encoders
- Aumente α para 0.98-0.99
- Considere adicionar filtro passa-baixa em `w_encoders`

### Problema: Encoders retornam 0 em baixas velocidades
**Causa**: Watchdog timer detecta parada (normal)
**Solução**: Isso é esperado. O filtro complementar ajuda suavizando a transição.

## Referências

- **Configuração do robô**: `include/config.h`
  - R = 0.0215 m (raio da roda)
  - L = 0.0825 m (distância entre rodas)
  
- **Implementação IMU**: `lib/imu/`
- **Implementação Encoders**: `lib/encoder/`

## Próximos Passos

1. Testar com diferentes valores de α
2. Comparar performance em diferentes modos de operação
3. Considerar implementar Filtro de Kalman (mais complexo, mas possivelmente melhor)
4. Integrar no controlador principal em `src/main.cpp`

---

**Criado em:** 13/11/2025  
**Autor:** Sistema de Controle UnBall
