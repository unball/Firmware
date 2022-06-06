#include "radio.hpp"
#include "control.hpp"

// TODO: Detectar falhas no rádio com radio.failureDetected e tentar recuperar

namespace Radio {

  dataStruct velocidades;

  RF24 radio(CE_PIN, CS_PIN);
  //RF24 *radio;

  const uint64_t channels[4] = { 0xABCDABCD71L, 0x544d52687CL, 0x644d52687CL, 0x744d52687CL };
  uint64_t channelEnvia;
  uint64_t channelRecebe;

  uint8_t robotNumber;

  volatile static uint32_t lastReceived;
  
  void setup(uint8_t robot, uint8_t sendChannel){
    //*radio = RF24(9, 10);
    robotNumber = robot; 
    channelRecebe=channels[robot];
    channelEnvia=channels[sendChannel];
    radio.begin();                           // inicializa radio
    radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
    radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
    radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

    radio.openWritingPipe(channelEnvia);        //escreve pelo pipe0 SEMPRE
    radio.openReadingPipe(1,channelRecebe);      //escuta pelo pipe1, evitar usar pipe0

    radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)

    // Põe o rádio para escutar mensagens
    radio.startListening();

    // Tempo inicial para a última mensagem recebida
    lastReceived = micros();

    // Ajusta o tamanho dos pacotes ao tamanho da mensagem
    radio.setPayloadSize(sizeof(reportStruct));
    // Põe o rádio para enviar mensagens
    radio.stopListening();

  }

  bool receiveData(double *v, double *w){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
    dataStruct data;
    if(radio.available()){
      radio.read(&data,sizeof(dataStruct));

      // Demultiplexa a mensagem e decodifica
      *v = data.v;

      // Se ultrapassar um certo valor, faz um spin a 360 rad/s
      //if (abs(data.w) > 32000) *w = (data.w >= 0 ? 1 : -1) * 360;
      //else *w = data.w * 64.0 / 32767;
      *w = data.w;

      // Atualiza o tempo de recebimento
      lastReceived = micros();
      return true;
    }
    return false;
  }

  void reportMessage(reportStruct *message){
    // Permite que não haja ACK
    radio.enableDynamicAck();

    // Envia a mensagem sem ACK
    radio.write(message, sizeof(reportStruct), 0);
  }

  // Checks if received any message from radio in the last threshold us
  bool isRadioLost(){
      if((micros() - lastReceived) > RADIO_THRESHOLD) return true;
      return false;
  }
} //end namespace
