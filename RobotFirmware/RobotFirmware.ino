#include <UnballLibraries.h>


int somaA=0;
void somaEA(void){
  somaA++;
}
int somaB=0;
void somaEB(void){
  somaB++;
}

void setup(void) {

  //if(TEENSY_DEBUG){
    Serial.begin(115200);
    //while( !Serial );
  //}

  Radio::Setup();
  Motor::Setup();
  Encoder::Setup();
  //Imu::Setup();
  Control::acc = 0;

  delay(5000);
  
  /*int pwmA=3;
  int ain1=4;
  int ain2=6;
  
  int stby=5;

  int pwmB=20;
  int bin1=14;
  int bin2=15;
  int led=13;
  int channelA=17;
  int channelB=7;
  
  pinMode(stby, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(channelA, INPUT);
  
  pinMode(pwmA, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  
  pinMode(pwmB, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  analogWrite(pwmA, 255);

  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmB, 255);
  
  digitalWrite(stby, HIGH);
  digitalWrite(led, HIGH);

  attachInterrupt(channelA, somaEA, RISING);
  attachInterrupt(channelB, somaEB, RISING);*/
}

long timer=0;
void loop(){
  Control::stand();
  //Serial.println("fim da stand");
  /*timer = micros() - timer;
  timer = micros();
  Serial.print((1000.0*somaA)/timer);Serial.print("\t");Serial.print((1000.0*somaB)/timer);Serial.print("\t");Serial.println(timer);
  somaA=0;
  somaB=0;
  delay(2);*/
}



