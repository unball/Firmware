#include "radio.hpp"

namespace Radio {

  dataStruct velocidades, report;

  RF24 radio(CE_PIN, CS_PIN);
  //RF24 *radio;

  /*const uint8_t channels [NUMBER_OF_ROBOTS+1][5] = {{0xAB, 0xCD, 0xAB, 0xCD, 0x71},
                                                    {0x54, 0x4D, 0x52, 0x68, 0x72},
                                                    {0x54, 0x4D, 0x52, 0x68, 0x73},
                                                    {0x54, 0x4D, 0x52, 0x68, 0x74},
                                                    {0x54, 0x4D, 0x52, 0x68, 0x75},
                                                    {0x54, 0x4D, 0x52, 0x68, 0x76}};*/
  const uint64_t channels[4] = { 0xABCDABCD71L, 0x544d52687CL, 0x644d52687CL, 0x744d52687CL };
  uint64_t channelEnvia;
  uint64_t channelRecebe;

  uint8_t robotNumber;
  
  void setup(uint8_t robot, uint8_t sendChannel){
    //*radio = RF24(9, 10);
    channelRecebe = channels[robot];
    channelEnvia = channels[sendChannel];
    radio.begin();                           // inicializa radio
    radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
    radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
    radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

    radio.openWritingPipe(channelEnvia);        //escreve pelo pipe0 SEMPRE
    radio.openReadingPipe(1,channelRecebe);      //escuta pelo pipe1, evitar usar pipe0

    radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
    radio.setPayloadSize(sizeof(dataStruct));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
                                                 //tamanho maximo de payload 32 BYTES

    radio.startListening();                 // Start listening
  }

  bool receiveData(dataStruct *data){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
     if(radio.available()){
      while(radio.available()){
        radio.read(data,sizeof(dataStruct));
      }
      return true;
     }
    return false;
  }

  void reportMessage(int message){
    report.A = robotNumber;
    report.B = message;
    radio.stopListening();
    radio.enableDynamicAck();                 //essa funcao precisa andar colada na função radio.write()
    radio.openWritingPipe(channelEnvia);
    for(int j=0; j<5; j++){
        radio.write(&report, sizeof(dataStruct), 1);  //lembrar que precisa enableDynamicAck antes
    }                                          // 1-NOACK, 0-ACK
    radio.startListening();
  }

  void reportMessage(Radio::vel message){
    radio.stopListening();
    radio.enableDynamicAck();                 //essa funcao precisa andar colada na função radio.write()
    radio.openWritingPipe(channelEnvia);

    /*Serial.print(message.time);
    Serial.print(", ");
    Serial.print(message.vel_A);
    Serial.print(", ");
    Serial.print(message.vel_B);
    Serial.print("\n");*/
    radio.write(&(message), sizeof(Radio::vel), 1);  //lembrar que precisa enableDynamicAck antes
                                          // 1-NOACK, 0-ACK
    radio.startListening();
  }
} //end namespace
