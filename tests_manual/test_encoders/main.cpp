#include <Arduino.h>

#define ENC_A_CH_A 35
#define ENC_A_CH_B 34
#define ENC_B_CH_A 18
#define ENC_B_CH_B 19

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENC_A_CH_A, INPUT);
  pinMode(ENC_A_CH_B, INPUT);
  pinMode(ENC_B_CH_A, INPUT);
  pinMode(ENC_B_CH_B, INPUT);

  Serial.println("🔁 Encoder test started.");
}

void loop() {
  Serial.print("Encoder A: A=");
  Serial.print(digitalRead(ENC_A_CH_A));
  Serial.print(" B=");
  Serial.print(digitalRead(ENC_A_CH_B));
  Serial.print(" | Encoder B: A=");
  Serial.print(digitalRead(ENC_B_CH_A));
  Serial.print(" B=");
  Serial.println(digitalRead(ENC_B_CH_B));
  delay(200);
}