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

    #define scale 1000
    int8_t triangular_incrementer = 1;

    void triangular_wave(int32_t *v1, int32_t *v2){
        static int32_t triangular_wave_cont;
        if(triangular_wave_cont >= scale){
            triangular_incrementer = -1;
        }
        else if(triangular_wave_cont <= -scale){
            triangular_incrementer = 1;
        }
        triangular_wave_cont += triangular_incrementer;
        *v1 = triangular_wave_cont*64/scale;
        *v2 = triangular_wave_cont*64/scale;
    }

    void sine_wave(int32_t *v1, int32_t *v2){
        static int32_t sine_wave_cont;
        if(sine_wave_cont >= 100000){
            sine_wave_cont = 0;
        }
        sine_wave_cont ++;
        *v1 = 25*sin(sine_wave_cont/400.0);
        *v2 = 25*sin(sine_wave_cont/400.0);
    }

    void square_wave(int32_t *v1, int32_t *v2){
        static uint32_t square_wave_cont;
        if(square_wave_cont > 500){
            wave_flag = -1*wave_flag;
            square_wave_cont = 0;
        }
        *v1 = 20*wave_flag;
        *v2 = 20*wave_flag;
        square_wave_cont++;
    }

    void step(int32_t *v1, int32_t *v2){
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

    #define DEADZONE 10
    int32_t deadzone(int32_t vin){
        if (vin!=0)
            return (vin > 0) ? vin+DEADZONE : vin-DEADZONE;
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

    double crossCoupledControl(double err){
        return 1*err;
    }

    //volatile int16_t ea1=0, ea2=0, ua1=0, ua2=0;
    void control(int32_t v1, int32_t v2){
        
        if(v1 || v2){
            Encoder::vel enc;
            enc = Encoder::encoder();

            double Cy = 1.0;
            double Cx = 1.0;
            double crossCoupledControlOut = crossCoupledControl(Cy*(v2 - enc.motorB)-Cx*(v1 - enc.motorA));

            double vVirt1 = v1 - Cx*crossCoupledControlOut;
            double vVirt2 = v2 + Cy*crossCoupledControlOut;

            double e1 = vVirt1 - enc.motorA;
            double e2 = vVirt2 - enc.motorB;

            int32_t controlA = (int32_t)control1(e1);
            int32_t controlB = (int32_t)control2(e2);

            Motor::move(0, deadzone(controlA));
            Motor::move(1, deadzone(controlB));

            #if false //CONTROL_DEBUG
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
            isRadioLost(true);
            #if CONTROL_DEBUG
            Serial.println("radioAvailable");
            #endif
        }
        //procedimento para indicar que o robo nao recebe mensagens nos ultimos 2 segundos(customizavel)
        if(isRadioLost(false)){
            #if CONTROL_DEBUG
            Serial.println("radioNotAvailable");
            Radio::reportMessage(1);
            #endif
            velocidades.A=20;
            velocidades.B=20;

            sine_wave(&velocidades.A, &velocidades.B);
            #if MOTOR_TEST
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