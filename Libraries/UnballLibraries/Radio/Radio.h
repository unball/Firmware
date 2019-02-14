#ifndef RADIO_H
#define RADIO_H
#include <Pins/Pins.h>


#include <SPI.h>
#include <RF24.h>

struct dataStruct{
    int16_t A=0;
    int16_t B=0;
};

dataStruct velocidades, report;

namespace Radio {
  RF24 radio(Pins::CE,Pins::CS);

  const uint64_t channels[4] = { 0xABCDABCD71LL, 0x544d52687CLL, 0x644d52687CLL, 0x744d52687CLL };
  uint64_t channelEnvia=channels[3];
  uint64_t channelRecebe=channels[robot_number];

  void Setup(){
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

  bool receivedata(dataStruct *data){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
     if(radio.available()){
      while(radio.available()){
        radio.read(data,sizeof(dataStruct));
      }
      return true;
     }
    return false;
  }

  void reportMessage(int message){
    report.A = robot_number;
    report.B = message;
    radio.stopListening();
    radio.enableDynamicAck();                 //essa funcao precisa andar colada na função radio.write()
    radio.openWritingPipe(channelEnvia);
    for(int j=0; j<5; j++){
        Serial.println(j);
        radio.write(&report, sizeof(dataStruct), 1);  //lembrar que precisa enableDynamicAck antes
    }                                          // 1-NOACK, 0-ACK
    radio.startListening();
  }
} //end namespace

#endif
