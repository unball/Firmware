#include <UnballLibraries.h>

void setup(void) {

  //if(TEENSY_DEBUG){
    Serial.begin(9600);
    //while( !Serial );
  //}

  Radio::Setup();
  Motor::Setup();
  Encoder::Setup();
  //Imu::Setup();
  Control::acc = 0;
  //delay(5000);
}

long timer=0;
void loop(){
  Control::stand();
}



