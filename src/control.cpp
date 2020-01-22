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
    
    /*
        Implementa uma onda triangular

        @param v1 Ponteiro para a velocidade do motor A
        @param v2 Ponteiro para a velocidade do motor B
        @param amplitude Valor em ticks/ms da amplitude da onda
        @param maxCount Número de chamadas a função necessário para realizar um quarto do ciclo da onda
    */
    void triangular_wave(int16_t *v1, int16_t *v2, int32_t amplitude, int32_t maxCount){
        static int32_t triangular_wave_cont;
        static int8_t state = 3, out;
        if(abs(triangular_wave_cont) >= maxCount){
            triangular_wave_cont = 0;
            state = (state+1)%4;
        }
        triangular_wave_cont += 1;
        out = (state % 2) * (state-2);
        *v1 = out*triangular_wave_cont*amplitude/maxCount;
        *v2 = out*triangular_wave_cont*amplitude/maxCount;
    }

    void sine_wave(int16_t *v1, int16_t *v2){
        static int32_t sine_wave_cont;
        if(sine_wave_cont >= 100000){
            sine_wave_cont = 0;
        }
        sine_wave_cont ++;
        *v1 = 25*sin(sine_wave_cont/400.0);
        *v2 = 25*sin(sine_wave_cont/400.0);
    }

    /*
        Implementa uma onda quadrada

        @param v1 Ponteiro para a velocidade do motor A
        @param v2 Ponteiro para a velocidade do motor B
        @param amplitude Valor em ticks/ms da amplitude da onda
        @param maxCount Número de chamadas a função necessário para realizar um quarto do ciclo da onda
    */
    void square_wave(int16_t *v1, int16_t *v2, int32_t amplitude, uint32_t maxCount){
        static uint32_t square_wave_cont;
        static int8_t state = 3, out;
        if(square_wave_cont > maxCount){
            square_wave_cont = 0;
            state = (state+1)%4;
        }
        out = (state % 2) * (state-2);
        *v1 = out*amplitude;
        *v2 = out*amplitude;
        square_wave_cont++;
    }

    void step(int16_t *v1, int16_t *v2){
        static uint32_t step_cont;
        static int8_t step_flag = 0;
        if(step_cont > 200){
            step_flag = 1;
        }
        
        if(step_cont>2000){
            step_flag = 0;
            
        }
        step_cont++;
        *v1 = 30*step_flag;
        *v2 = 30*step_flag;
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

    int32_t deadzone(int32_t vin, int32_t up, int32_t down){
        if (vin!=0)
            return (vin > 0) ? vin+up : vin-abs(down);
        return 0;
    }

    double saturation(double vin){
        return min(max(vin, -255.0), 255.0);
    }

    double control1(double err){
        static double old_err;
        static double old_out;
        double out = (  1.9077 * (err - 0.9418  *  old_err) + old_out);
        old_err = err -  (saturation(out)-out);
        old_out = out; //=  (abs(out) < 255)? out : 0;
        return out;
    }

    double control2(double err){
        static double old_err;
        static double old_out;
        double out =  (   1.9077 * (err  - 0.9418 *  old_err) + old_out);
        old_err = err - (saturation(out)-out);
        old_out = out; //(abs(out) < 255)? out : 0;;
        return out;
    }

    //volatile int16_t ea1=0, ea2=0, ua1=0, ua2=0;
    void control(int32_t v1, int32_t v2){
        
        if(v1 || v2){
            Encoder::vel enc;
            enc = Encoder::encoder();
            //Serial.printf("%lf\t%lf\n", enc.motorA, enc.motorB);

            /*double Cy = 1.0;
            double Cx = 1.0;
            double crossCoupledControlOut = crossCoupledControl(Cy*(v2 - enc.motorB)-Cx*(v1 - enc.motorA));

            double vVirt1 = v1 - Cx*crossCoupledControlOut;
            double vVirt2 = v2 + Cy*crossCoupledControlOut;*/

            // Erro de cada motor
            double e1 = v1 - enc.motorA;
            double e2 = v2 - enc.motorB;

            // Erro de malha acoplada
            double cccError = 3.0/sqrt(v1*v1 + v2*v2) * (-v2 * enc.motorA + v1 * enc.motorB);
            if (v1 < 0 && v2 < 0){
                cccError *= -1;
            }
            //Serial.println(cccError);

            // Controle digital
            int32_t controlA = (int32_t)control1(e1+cccError);
            int32_t controlB = (int32_t)control2(e2-cccError);

            // Passa para a planta a saída do controle digital e da malha acoplada
            Motor::move(0, deadzone(controlA, 7, -7));
            Motor::move(1, deadzone(controlB, 7, -7));

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
        static Radio::vels velocidades;    
        
        #if CONTROL_ID
            static const size_t size = CONTROL_ID_BUFFER_SIZE;
            static Radio::reportStruct messages[size];
            static size_t index = 0;
            
            // Obtém velocidades do encoder
            Encoder::vel enc = Encoder::encoder();

            // Escolhe quais serão as entradas da planta
            #if   (CONTROL_ID_MODE == CONTROL_ID_MODE_DEADZONE)
                triangular_wave(&velocidades.A, &velocidades.B, 64, 500);

                // Alimenta a planta com as entradas
                Motor::move(0, velocidades.A);
                Motor::move(1, velocidades.B);

            #elif (CONTROL_ID_MODE == CONTROL_ID_MODE_ID)
                square_wave(&velocidades.A, &velocidades.B, 20, 750);

                // Alimenta a planta com as entradas
                Motor::move(0, deadzone(velocidades.A, 7, -7));
                Motor::move(1, deadzone(velocidades.B, 7, -6));

            #elif (CONTROL_ID_MODE == CONTROL_ID_MODE_VALIDATION)
                square_wave(&velocidades.A, &velocidades.B, 20, 750);

                
                // Erro de malha acoplada
                double cccError = 20.0/sqrt(velocidades.A*velocidades.A + velocidades.B*velocidades.B) * (-velocidades.B * enc.motorA + velocidades.A * enc.motorB);
                if (velocidades.A < 0 && velocidades.B < 0){
                    cccError *= -1;
                }

                // Alimenta a planta com as entradas
                Motor::move(0, deadzone((int32_t)control1(velocidades.A-enc.motorA+cccError), 7, -7));
                Motor::move(1, deadzone((int32_t)control2(velocidades.B-enc.motorB-cccError), 7, -6));
            #endif

            // Compõe a mensagem a ser enviada pelo rádio
            Radio::reportStruct message = {
                .time = micros(),
                .va = velocidades.A, 
                .vb = velocidades.B, 
                .enca = enc.motorA, 
                .encb = enc.motorB
            };
            messages[index] = message;

            #if   (CONTROL_ID_TRANSFER == CONTROL_ID_TRANSFER_SERIAL)

                // Reporta mensagem via serial
                Serial.printf("%d %d %d %lf %lf\n", message.time, message.va, message.vb, message.enca, message.encb);

            #elif (CONTROL_ID_TRANSFER == CONTROL_ID_TRANSFER_RADIO)

                // Envia a mensagem pelo rádio
                if(index == size-1){
                    Motor::stop();
                    for(size_t i=0 ; i<index+1 ; i++){
                        Radio::reportMessage(&messages[i]);
                    }
                    index = 0;
                }
                else index++;

            #endif
            
        #else
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

                sine_wave(&velocidades.A, &velocidades.B);
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
                Radio::radio.flush_rx();
                Led::red();
            }
        #endif
    }

}//end namespace