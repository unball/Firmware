#include "radio.hpp"

// TODO: Detectar falhas no rádio com radio.failureDetected e tentar recuperar

namespace Radio {

  dataStruct velocidades;

  RF24 radio(CE_PIN, CS_PIN);

  const uint64_t channels[3] = { 0xABCDABCD71LL, 0x544d52687CLL, 0x644d52687CLL};
  uint64_t channelRecebe;

  uint8_t robotNumber;

  volatile static uint32_t lastReceived;
  
  void setup(uint8_t robot, uint8_t sendChannel){
    robotNumber = robot;
    channelRecebe=channels[robot];
    if (!radio.begin()) {
      Serial.println("radio is not responding");
    } else {
      Serial.println("******radio OK*****");
    }
    radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
    radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
    radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

    radio.openReadingPipe(1,channelRecebe);      //escuta pelo pipe1, evitar usar pipe0

    radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)

    // Põe o rádio para escutar mensagens
    radio.startListening();

    // Tempo inicial para a última mensagem recebida
    lastReceived = micros();

    // Ajusta o tamanho dos pacotes ao tamanho da mensagem
    radio.setPayloadSize(sizeof(dataStruct));
  }

  bool receiveData(double *vl, double *vr){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
    dataStruct data;
    if(radio.available()){
      radio.read(&data,sizeof(dataStruct));

      *vl = data.vl;
      *vr = data.vr;

      // Atualiza o tempo de recebimento
      lastReceived = micros();
      return true;
    }
    return false;
  }

  // Checks if received any message from radio in the last threshold us
  bool isRadioLost(){
      if((micros() - lastReceived) > RADIO_THRESHOLD) return true;
      return false;
  }
}