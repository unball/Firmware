#include "radio.hpp"
#include "control.hpp"

namespace Radio {

  dataStruct velocidades;

  RF24 radio(CE_PIN, CS_PIN);
  //RF24 *radio;

  const uint64_t channels[4] = { 0xABCDABCD71L, 0x544d52687CL, 0x644d52687CL, 0x744d52687CL };
  uint64_t channelEnvia;
  uint64_t channelRecebe;

  uint8_t robotNumber;
  
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

    #if !CONTROL_ID
      // Ajusta o tamanho dos pacotes ao tamanho da mensagem
      radio.setPayloadSize(sizeof(dataStruct)); 
    #else
      // Ajusta o tamanho dos pacotes ao tamanho da mensagem
      radio.setPayloadSize(sizeof(reportStruct));
      // Põe o rádio para enviar mensagens
      radio.stopListening();
    #endif
  }

  bool receiveData(vels *ret){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
    dataStruct data;
     if(radio.available()){
      //while(radio.available()){
        radio.read(&data,sizeof(dataStruct));
      //}
      ret->A = data.A[robotNumber];
      ret->B = data.B[robotNumber];
      return true;
     }
    return false;
  }

  void reportMessage(reportStruct *message){
    // Permite que não haja ACK
    radio.enableDynamicAck();

    // Envia a mensagem sem ACK
    radio.write(message, sizeof(reportStruct), 1);
  }
} //end namespace
