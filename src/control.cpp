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
        #if CONTROL_DEBUG
        Serial.print("TIME: ");
        Serial.println(timer);
        #endif
        cicle_time = millis();
    }

    //checks if received any message from radio in the last threshold us
    bool isRadioLost(bool received, uint32_t threshold = 2000000){
        static uint32_t lastReceived = micros();
        if(received) lastReceived = micros();
        else if((micros() - lastReceived) > threshold) return true;
        return false;
    }

    void TestWave(int32_t *v1, int32_t *v2){
        #if (wave == 1)
            angulo++;
            #if CONTROL_DEBUG
            if(angulo>720*4){
                Serial.println("#");
            }
            #endif
            *v1 = (uint32_t)40*sin(angulo*(PI/180)*2);
            *v2 = *v1;
        #elif (wave == 2)
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
        #elif (wave == 3)
            *v1 = 70;
            *v2 = 70;
        #endif
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
            Serial.print(v1); Serial.println(v2);
            Encoder::vel enc;
            enc = Encoder::encoder();

            double e1 = v1 - enc.motorA;
            double e2 = v2 - enc.motorB;


            int32_t power1 = 0;// = ((2809*e1 - 2317*ea1)>>10) + ua1;
            ea1 = e1;
            ua1 = power1;

            int32_t power2 = 0;// = ((2809*e2 - 2317*ea2)>>10) + ua2;
            ea2 = e2;
            ua2 = power2;
            power1 = (int32_t)e1*10;
            power2 = (int32_t)e2*10;

            Motor::move(0, power1);
            Motor::move(1, power2);
            #if false //CONTROL_DEBUG
            Encoder::encoder();
            Serial.println("$");
            Serial.println(v1);
            //Serial.println(Encoder::contadorA_media);
            //Serial.println(Encoder::contadorB_media);
            TimeOfCicle();
            #endif
        }
        else{
            Encoder::resetEncoders();
            stopRobot();
            #if CONTROL_DEBUG
            //Serial.print(Encoder::contadorA_media);Serial.print("\t");
            //Serial.println(Encoder::contadorB_media);Serial.print("\t");
            #endif
        }
    }

    void stand() {
        static Radio::dataStruct velocidades;    
        if(Radio::receiveData(&velocidades)) {
            #if CONTROL_DEBUG
            Serial.println("radioAvailable");
            Serial.println("Received velocities:");
            Serial.print("=>\tvA:");Serial.print(velocidades.A);
            Serial.print("\t");
            Serial.print("vB:");Serial.println(velocidades.B);
            #endif
            isRadioLost(true);
        }
        //procedimento para indicar que o robo nao recebe mensagens nos ultimos 2 segundos(customizavel)
        if(isRadioLost(false)){
            #if CONTROL_DEBUG
            Serial.println("Radio Lost");
            Radio::reportMessage(1);
            #endif
            velocidades.A=20;
            velocidades.B=-20;

            #if MOTOR_TEST
            TestWave(&velocidades.A, &velocidades.B);
            Led::blue();
            #endif

            control(velocidades.A, velocidades.B);
            //motorId();
        }
        else {
            //motorId();
            control(velocidades.A, velocidades.B);
            Led::red();
        }
    }

}//end namespace