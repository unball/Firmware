#include <control.hpp>

namespace Control {

    int32_t acc = 0;
    int32_t ctr = 0;

    //variaveis de teste
    int8_t wave_flag=1;
    int16_t angulo=0;
    int64_t square_cont=0, cont=0;

    uint32_t timer=0;
    
    void stopRobot() {
        Motor::stop();
    }

    
    //verifica e imprime o tempo de duração de um ciclo
    void TimeOfCicle(){
        static uint32_t cicle_time = millis();
        cicle_time = millis() - cicle_time;
        timer = timer + cicle_time;
        //Serial.print("TIME: ");
        #if CONTROL_DEBUG
        Serial.println(timer);
        #endif
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
            #if CONTROL_DEBUG
            if(angulo>720*4){
                Serial.println("#");
            }
            #endif
            *v1 = 40*sin(angulo*(PI/180)*2);
            *v2 = *v1;
        }
        else if(wave == 2){
            cont+=1;
            #if CONTROL_DEBUG
            if(square_cont > 8){
                Serial.println("#");
            }
            #endif
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
            Encoder::vel enc;
            enc = Encoder::encoder();

            int32_t e1 = v1 - enc.motorA;
            int32_t e2 = v2 - enc.motorB;

            int32_t power1 = ((2809*e1 - 2317*ea1)>>10) + ua1;
            ea1 = e1;
            ua1 = power1;

            int32_t power2 = ((2809*e2 - 2317*ea2)>>10) + ua2;
            ea2 = e2;
            ua2 = power2;

            Motor::move(0, power1);
            Motor::move(1, power2);
            #if CONTROL_DEBUG
            Encoder::encoder();
            Serial.println("$");
            Serial.println(v1);
            Serial.println(Encoder::contadorA_media);
            Serial.println(Encoder::contadorB_media);
            TimeOfCicle();
            #endif
        }
        else{
            Encoder::resetEncoders();
            stopRobot();
            #if CONTROL_DEBUG
            Serial.print(Encoder::contadorA_media);Serial.print("\t");
            Serial.println(Encoder::contadorB_media);Serial.print("\t");
            #endif
        }
    }

    void stand() {
        static Radio::dataStruct velocidades;    
        if(Radio::receiveData(&velocidades)) {
            acc=0;
            #if CONTROL_DEBUG
            Serial.println("radioAvailable");
            if(frame_rate()){
                Radio::reportMessage(2);
            }
            #endif
        }
        //procedimento para indicar que o robo nao recebe mensagens nas ultimas 2500 iteracoes
        if(radioNotAvailableFor(2500)){
            #if CONTROL_DEBUG
            Serial.println("radioNotAvailable");
            Radio::reportMessage(1);
            #endif
            double vA=40, vB=-40;
            if(MOTOR_TEST){
                TestWave(&vA, &vB);
                Led::blue();
            }
            control(vA, vB);
            //motorId();
        }
        else {
            //motorId();
            control(velocidades.A, velocidades.B);
            Led::red();
        }
    }

}//end namespace