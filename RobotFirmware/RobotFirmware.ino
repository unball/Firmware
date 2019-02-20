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
