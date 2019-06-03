#ifndef CONTROL_H
#define CONTROL_H

#include <Motor/Motor.h>
#include <Encoder/Encoder.h>
#include <Radio/Radio.h>
//#include <LedRGB/LedRGB.h>
//#include <Imu/Imu.h>

#define MOTOR_TEST false    //define se está ou não fazendo o teste nos motores
#define DEBUG_CONTROl false
#define wave 1              // sine = 1 -- square = 2 -- step = 3

typedef struct{
    double lin;
    double ang;
} linAng;

namespace Control {
    //unsigned long cicle_time=0;
    int acc = 0;
    int ctr = 0;
    bool start_flag = true;

    //variaveis de teste
    int wave_flag=1;
    int angulo=0;
    long square_cont=0, cont=0;


    void stopRobot() {
        Motor::stop(0);
        Motor::stop(1);
    }

    uint32_t timer=0;
    //verifica e imprime o tempo de duração de um ciclo
    void TimeOfCicle(){
        static uint32_t cicle_time = millis();
        cicle_time = millis() - cicle_time;
        timer = timer + cicle_time;
        //Serial.print("TIME: ");
        Serial.println(timer);
        cicle_time = millis();
    }

    bool radioNotAvailableFor(int numberOfCicles) {
        if(acc<numberOfCicles+10)
            acc++;
        return acc>numberOfCicles;
    }

    void TestWave(double *v1, double *v2){
        if(wave == 1){
            angulo++;
            if(angulo>720*4){
            //Serial.println("#");
            }
            *v1 = 40*sin(angulo*(PI/180)*2);
            *v2 = *v1;
        }
        else if(wave == 2){
            cont+=1;
            if(square_cont > 8){
            Serial.println("#");
            }
            if(cont>200){
            wave_flag = -1*wave_flag;
            cont = 0;
            square_cont+=1;
            }
            *v1 = (*v1)*wave_flag;
            *v2 = -1*(*v2)*wave_flag;
        }
        else if(wave == 3){
            *v1 = 70;
            *v2 = 70;
        }
    }

    bool frame_rate(){
        ctr++;
        if(ctr == 100){
            ctr = 0;
            return true;
        }
        else{
            return false;
        }
    }

    void Turbo(int turboA, int turboB){
        if(turboA>0 && turboB>0){
            Motor::move(0, 255);
            Motor::move(1, 255);
        }
        else if(turboA<0 && turboB<0){
            Motor::move(0, -255);
            Motor::move(1, -255);
        }
        else if(turboA>0 && turboB<0){
            Motor::move(0, 255);
            Motor::move(1, -255);
        }
        else{
            Motor::move(0, -255);
            Motor::move(1, 255);
        }
    }

    volatile int16_t ea1=0, ea2=0, ua1=0, ua2=0;
    void control(int32_t v1, int32_t v2){
        if(v1 || v2){
            Encoder::encoder();

            int32_t e1 = v1 - Encoder::contadorA_media;
            int32_t e2 = v2 - Encoder::contadorB_media;

            int32_t power1 = ((2809*e1 - 2317*ea1)>>10) + ua1;
            ea1 = e1;
            ua1 = power1;

            int32_t power2 = ((2809*e2 - 2317*ea2)>>10) + ua2;
            ea2 = e2;
            ua2 = power2;

            //Encoder::encoder();
            //Serial.println("$");
            //Serial.println(v1);
            //Serial.println(Encoder::contadorA_media);
            //Serial.println(Encoder::contadorB_media);
            Motor::move(0, power1);
            Motor::move(1, power2);
            //TimeOfCicle();
        }
        else{
            Encoder::resetEncoders();
            stopRobot();
            //Serial.print(Encoder::contadorA_media);Serial.print("\t");
            //Serial.println(Encoder::contadorB_media);Serial.print("\t");
        }
  }

    //method for motor identification
    void motorId(){
        for(uint16_t i = 0; i < 2901; i++){
                    Encoder::encoder();
                    Serial.println("$");
                    Serial.println(100);
                    Serial.println(Encoder::contadorA);
                    Serial.println(Encoder::contadorB);
                    Motor::move(0, 100);
                    Motor::move(1, 100);
                    TimeOfCicle();
                }
        for(uint16_t i = 0; i < 100; i++){
                    Encoder::encoder();
                    Serial.println("$");
                    Serial.println(100);
                    Serial.println(Encoder::contadorA);
                    Serial.println(Encoder::contadorB);
                    Motor::move(0, 100);
                    Motor::move(1, 100);
                    TimeOfCicle();
                }
                Serial.println("#");
    }

  void stand() {
    if(Radio::receivedata(&velocidades)) {      //velocidades vem de Radio.h
        //Serial.println("radioAvailable");
       acc=0;
       if(frame_rate()){
           //Radio::reportMessage(2);
      }
    }
    //procedimento para indicar que o robo nao recebe mensagens nas ultimas 2500 iteracoes
    if(radioNotAvailableFor(2500)){
        //Serial.println("radioNotAvailable");
        //Radio::reportMessage(1);
        //led::green();
        double vA=40, vB=-40;
        if(MOTOR_TEST){
            TestWave(&vA, &vB);
            //led::blue();
        }
        control(vA, vB);
        //motorId();
    }
    else {
      //control(500, 500);

        if(start_flag){
            start_flag = false;

            //Radio::reportMessage(2);
            //Serial.println("aqui");
        }

        //motorId();
        control(velocidades.A, velocidades.B);
        //led::red();
    }
  }

}//end namespace

#endif
