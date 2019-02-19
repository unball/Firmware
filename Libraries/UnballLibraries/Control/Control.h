#ifndef CONTROL_H
#define CONTROL_H

#include <Motor/Motor.h>
#include <Encoder/Encoder.h>
#include <Radio/Radio.h>
#include <LedRGB/LedRGB.h>
#include <Imu/Imu.h>

#define MOTOR_TEST false    //define se está ou não fazendo o teste nos motores
#define DEBUG_CONTROl false
#define wave 2              // sine = 1 -- square = 2 -- step = 3

typedef struct{
    double lin;
    double ang;
} linAng;

namespace Control {
    long errorA_i=0;
    long errorB_i=0;
    double errorLinI = 0;
    double errorAngI = 0;
    double errorLinDAnt = 0;
    double errorAngDAnt = 0;
    long errorA_d_ant=0;
    long errorB_d_ant=0;
    long commandA_media=0;
    long commandB_media=0;
    int sat_count = 0;
    unsigned long cicle_time=0;
    bool bateria_fraca;
    int acc = 0;
    int ctr = 0;
    bool start_flag = true;

    //variaveis de teste
    int wave_flag=1;
    int angulo=0;
    long square_cont=0, cont=0;


    void stopRobot() {
        Motor::stop();
    }

    //verifica e imprime o tempo de duração de um ciclo
    void TimeOfCicle(){
        cicle_time = millis() - cicle_time;
        Serial.print("TIME: ");
        Serial.println(cicle_time);
        cicle_time = millis();
    }

    linAng vel;
    void computVel(){
        vel.ang = Imu::imuData.gyro.z;
        vel.lin = vel.lin + (((cos(Imu::imuData.roll)*2*Imu::imuData.accel.y) - 0.09785)*cicle_time/1000);
    }

    void control(double velocidadeA, double velocidadeB){
        if(velocidadeA || velocidadeB){
            TimeOfCicle();
            Imu::imuRead();
            computVel();
            if(DEBUG_CONTROl){
                TimeOfCicle();
                Serial.print("motor0: ");
                //Serial.print(Encoder::presentVelA);
                Serial.print("//");
                //Serial.print(Encoder::contadorA_media);
                Serial.print("  motor1: ");
                //Serial.print(Encoder::presentVelB);
                Serial.print("//");
                //Serial.print(Encoder::contadorB_media);
            }
;
            double errorLin = velocidadeA - vel.lin;
            double errorAng = velocidadeB - vel.ang;

            errorLinI += errorLin;
            errorAngI += errorAng;

            double errorLinD = errorLin - errorLinDAnt;
            double errorAngD = errorAng - errorAngDAnt;
            errorLinDAnt = errorLin;
            errorAngDAnt = errorAng;

            if(DEBUG_CONTROl){
                Serial.print(" error:");
                //Serial.print(errorA);
                Serial.print("||");
                //Serial.print(errorB);
            }

            long kp_a;
            long ki_a;
            long kd_a;

            long kp_b;
            long ki_b;
            long kd_b;

            if(robot_number == 0){
                kp_a=1100;            //placa 2 = 1100;      //placa 6 = 1100      //placa 3 = 1100
                ki_a=35;              //placa 2 = 35;        //placa 6 = 38        //placa 3 = 40
                kd_a=2000;            //placa 2 = 2000;      //placa 6 =1800;      //placa 3 = 1500

                kp_b=1100;            //placa 2 = 1100;       //placa 6 = 1200     //placa 3 = 1100
                ki_b=35;              //placa 2 = 35;         //placa 6 = 40       //placa 3 = 40
                kd_b=2500;            //placa 2 = 2500;       //placa 6 = 1800     //placa 3 = 1500
            }
            else if(robot_number == 1){
                kp_a=1100;            //placa 2 = 1100;      //placa 6 = 1100      //placa 3 = 1100
                ki_a=40;              //placa 2 = 35;        //placa 6 = 38        //placa 3 = 40
                kd_a=1500;            //placa 2 = 2000;      //placa 6 =1800;      //placa 3 = 1500

                kp_b=1100;            //placa 2 = 1100;       //placa 6 = 1200     //placa 3 = 1100
                ki_b=40;              //placa 2 = 35;         //placa 6 = 40       //placa 3 = 40
                kd_b=1500;            //placa 2 = 2500;       //placa 6 = 1800     //placa 3 = 1500
            }
            else if(robot_number == 2){
                kp_a=1100;            //placa 2 = 1100;      //placa 6 = 1100      //placa 3 = 1100
                ki_a=38;              //placa 2 = 35;        //placa 6 = 38        //placa 3 = 40
                kd_a=1800;            //placa 2 = 2000;      //placa 6 =1800;      //placa 3 = 1500

                kp_b=1200;            //placa 2 = 1100;       //placa 6 = 1200     //placa 3 = 1100
                ki_b=40;              //placa 2 = 35;         //placa 6 = 40       //placa 3 = 40
                kd_b=1800;            //placa 2 = 2500;       //placa 6 = 1800     //placa 3 = 1500
            }

            //Saturação do errorA_i
            double Saturacao_erroA_i = 200000/ki_a;
            if(errorLinI > Saturacao_erroA_i){
                errorLinI = Saturacao_erroA_i;
            }
            else if(errorLinI < (-1)*Saturacao_erroA_i){
                errorLinI = (-1)*Saturacao_erroA_i;
            }
            //Saturação do errorB_i
            double Saturacao_erroB_i = 200000/ki_b;
            if(errorAngI > Saturacao_erroB_i){
                errorAngI = Saturacao_erroB_i;
            }
            else if(errorAngI < (-1)*Saturacao_erroB_i){
                errorAngI = (-1)*Saturacao_erroB_i;
            }

                                   //Proporcional     Integrativo       Derivativo
            double lin = ( (kp_a*errorLin) + (ki_a*errorLinI) + (kd_a*errorLinD) )/1000;
            double ang = ( (kp_b*errorAng) + (ki_b*errorAngI) + (kd_b*errorAngD) )/1000;


            if(DEBUG_CONTROl){
                //Serial.print("  errorA_i: ");Serial.print(errorA_i);
                //Serial.print("  errorB_i: ");Serial.print(errorB_i);
            }


            long commandA = (long) (lin - 0.5*ang*0.075)/0.035;
            long commandB = (long) (lin + 0.5*ang*0.075)/0.035;

            //testar filtro passa baixa
            /*float km = 0.9;
            commandA_media = km*commandA_media + (1-km)*commandA;
            commandB_media = km*commandB_media + (1-km)*commandB;
            commandA = commandA_media;
            commandB = commandB_media;*/


            //Teste para verificação dos motores
            if(MOTOR_TEST){
                Serial.println("$");
                Serial.println(velocidadeA);
                //Serial.println(Encoder::contadorA_media);
                //Serial.println(Encoder::contadorB_media);
                //Serial.println(errorA);
                //Serial.println(errorB);
            }

            if(commandA > 255){
                commandA = 255;
            }
            else if(commandA < -255){
                commandA = -255;
            }
            if(commandB > 255){
                commandB = 255;
            }
            else if(commandB < -255){
                commandB = -255;
            }

            Motor::move(0, commandA);
            Motor::move(1, commandB);

            if(DEBUG_CONTROl){
                Serial.print("   commands ");
                Serial.print(commandA);Serial.print("//");
                Serial.print(" ");Serial.println(commandB);
            }

        }else{
            //Encoder::resetEncoders();
            stopRobot();
        }
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
        Serial.println("#");
        }
        *v1 = 700*sin(angulo*(PI/180)*2);
        *v2 = 700*sin(angulo*(PI/180)*2);
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
        *v1 = 500;
        *v2 = 500;
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

  void stand() {
      Serial.println("stand");
    if(Radio::receivedata(&velocidades)) {
        Serial.println("radioAvailable");
       acc=0;
       if(frame_rate()){
           //Radio::reportMessage(2);
      }
    }
    //procedimento para indicar que o robo nao recebe mensagens nas ultimas 20000 iteracoes
    if(radioNotAvailableFor(20000)){
        //Serial.println("radioNotAvailable");
        //Radio::reportMessage(1);
        //led::green();
        double vA=0.5, vB=-0.5;
        if(MOTOR_TEST){
            TestWave(&vA, &vB);
            //led::blue();
        }
        control(vA, vB);
    }
    else {
      //control(500, 500);

        if(start_flag){
            start_flag = false;

            //Radio::reportMessage(2);
            Serial.println("aqui");
        }

        control(velocidades.A, velocidades.B);
        led::red();
    }
  }

}//end namespace

#endif
