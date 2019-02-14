#define robot_number 1 //Define qual robô esta sendo configurado [0->placa 2] [1->placa 3] [2->placa 6]
#define TEENSY_DEBUG false
#include <UnballLibraries.h>

void setup(void) {
  if(TEENSY_DEBUG){
    Serial.begin(115200);
    while( !Serial );
  }

  Radio::Setup();
  Motor::Setup();
  Encoder::Setup();
  Imu::Setup();
  Control::acc = 0;
}

void loop(){
  Control::stand();
}
