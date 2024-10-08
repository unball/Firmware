#include <control.hpp>
#include <encoder.hpp>

namespace Control {

    using namespace Waves;

    double erro = 0;
    double err_sum = 0;
    double last_err = 0;

    double kp = 0.54;
    double ki = 0.10;
    double kd = -0.08;

    /*
        Função que corrige a deadzone de um motor

        @param vin Entrada a ser corrigida
        @param up Valor positivo da deadzone
        @param down Valor negativo da deadzone (não importa o sinal, visto que a função usa abs(down))
    */
    int32_t deadzone(int32_t vin, int32_t up, int32_t down){
        if (vin!=0)
            return (vin > 0) ? vin+up : vin-abs(down);
        return 0;
    }

    /*
        Função que satura uma entrada para valores entre -255 e 255
    */
    inline double saturation(double vin){
        return min(max(vin, -255.0), 255.0);
    }

    /*
        Função que recebe velocidades dos encoders em ticks/ms e converte para velocidade linear em m/s
    */
    inline double linSpeed(Encoder::vel enc){
        return (enc.motorA+enc.motorB)/2 * TICKS2METER;
    }

    /*
        Função que recebe velocidade angular do IMU em º/ms e converte para velocidade angular em rad/s
    */
    inline double angSpeed(double imuW){
        return imuW * PI / 180;
    }

    /*
        Lê os encoders e a velocidade angular do IMU filtrada em primeira ordem e
        retorna a velocidade linear em m/s e a velocidade angular em rad/s por referência
    */
    void readSpeeds(double *w, double *v){
        // *w = angSpeed(-IMU::get_w());
        *w = -IMU::get_w();
         
        *v = linSpeed(Encoder::encoder());
    }

    double abs(double a){
        if (a > 0){
            return a;
        }
        else{
            return a*(-1);
        }
         
    }

    /*
        Implementa um PI digital com anti-windup para o motor A

        @param Recebe o erro
    */
   
    Control_cee CEE(double v, double eV, double w, double eW){

        double x[2];
        double u[2];
        Control_cee y;

        double A[2][2];
        double B[2][2];
        double C[2][2];
        double D[2][2];

        //inicialia A
        A[0][0] = 1/r;
        A[0][1] = L/(2*r);
        A[1][0] = 1/r;
        A[1][1] = -L/(2*r);

        //inicializa B
        B[0][0] = r/2;
        B[0][1] = r/2;
        B[1][0] = r/L;
        B[1][1] = -r/L;

        //inicializa c
        C[0][0] = 1;
        C[0][1] = 0;
        C[1][0] = 0;
        C[1][1] = 1;


        //inicialia D
        D[0][0] = 0;
        D[0][1] = 0;
        D[1][0] = 0;
        D[1][1] = 0;

        double aux[2][2];
        double aux2[2][2];
        double new_x[2];

        // inicializa vetores de estados e de entradas
        u[0] = v;
        u[1] = w;

        x[0] = eV;
        x[1] = eW;

        // resolve new_x = A*x + B*u
        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++){
                aux[i][j] = B[i][j]*u[j];
            }  
        }

        for (int j = 0; j < 2; j++){
            aux[0][j] += aux[1][j];
            aux[1][j] = 0;
        } 

        for (int i = 0; i < 2; i++){
            for (int j = 0; i < 2; i++){
                aux2[i][j] = A[i][j]*x[j];
            }  
        }

        for (int j = 0; j < 2; j++){
            aux2[0][j] += aux2[1][j];
            aux2[1][j] = 0;
        } 

        for (int j = 0; j < 2; j++){
            new_x[j] = aux[0][j] + aux2[0][j];
        }

        // resolve new_x = C*y + D*u
        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++){
                aux[i][j] = D[i][j]*u[j];
            }  
        }

        for (int j = 0; j < 2; j++){
            aux[0][j] += aux[1][j];
            aux[1][j] = 0;
        } 

        for (int i = 0; i < 2; i++){
            for (int j = 0; i < 2; i++){
                aux2[i][j] = C[i][j]*new_x[j];
            }  
        }

        for (int j = 0; j < 2; j++){
            aux2[0][j] += aux2[1][j];
            aux2[1][j] = 0;
        } 

        y.v = aux[0][0] + aux2[0][0];
        y.w = aux[0][1] + aux2[0][1];

        return y;
        
    }


    /// @brief Move the motors without control, based on the radius of the wheel and distance between them.
    /// @param v Linear velocity of the robot
    /// @param w Angualr velocity of the robot
    void speed2motors(double v, double w){
        
        if (v == 0 && w == 0){
            Motor::stop();
            return;
        }
        
        // Calculates the angular speed of rotation to each wheel
        int32_t vr = (v + (L/2)*w) / r;
        int32_t vl = (v - (L/2)*w) / r;

        vr = (int32_t)saturation((deadzone(vr, motor_deadzone, -motor_deadzone)));
        vl = (int32_t)saturation((deadzone(vl, motor_deadzone, -motor_deadzone)));

        Motor::move(0, vr);
        Motor::move(1, vl);
    }

    /*
        Implementa a malha de controle baixo nível

        @param v Velocidade linear de referência em m/s
        @param w Velocidade angular de referência em rad/s
        @param currW Velocidade angular medida por algum sensor (IMU) em rad/s
        @param *erro ponteiro para a variavel global de erro
    */
    void control(double v, double w, double currW, double currV){
        //TODO: deadzone?
        if (v == 0 && w == 0){
            Motor::stop();
            return;
        }
        
        // Angular and linear velocity error
        double eW = w - currW;
        double eV = v - currV;
        
        Control_cee saida_cee;
        //w = PID(v, eW);
        saida_cee = CEE(v,eV, w, eW);

        speed2motors(saida_cee.v, saida_cee.w);


    }


    void stand(){

        // Velocities to be read by Wi-Fi, they are static in case Wifi::receiveData does not receive anything, it keeps the previous velocity
        static double v = 0;
        static double w = 0;
        int16_t v_int = 0;
        int16_t w_int = 0;
       

        // Velocidades atuais medidas por sensores
        double currW;
        double currV;
        
        // Lê velocidades pelo Wifi
        Wifi::receiveData(&v_int, &w_int);

        //demutiplexa velocidades
        v = 1;//((float)v_int) * 2.0 / 32767;
        w  = 0;//((float)w_int) * 64.0 / 32767;

        if(false){
            err_sum = 0;
            last_err = 0;

			v = Waves::sine_wave();
			w = 0;

            Motor::move(0, v);
            Motor::move(1, v);
        }
        else{
            // Execute the control normally with the reference velocities
            // Read the velocities through the sensor
            readSpeeds(&currW, &currV);
            // Execute the control loop
            control(v, w, currW, currV);
        }
    }

    double test(){
        // Velocidade
        static double v = 0;
        static double w = 0;
        
        // Velocidades atuais medidas por sensores
        double currW = 0;

        //zera erro
        erro = 0;


        static int32_t previous_t;
        static int32_t t;

        //faz um quadrado de frente
        for (int i = 0; i < 4; i++){
            previous_t = millis();
            while (t - previous_t < 700 ){
                t = millis();
                //rotina de controle anda de frente
                v = 0.2;
                w = 0;
                //readSpeeds(&currW);
                //control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
                
            }
            
            previous_t = millis();
            while (t - previous_t < 314 ){
                t = millis();
                //rotina de virar
                v = 0;
                w = 5;
                //readSpeeds(&currW);
                //control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }
        }

        while (t - previous_t < 300 ){
            t = millis();
            //rotina de virar
            v = 0;
            w = 0;
            //readSpeeds(&currW);
            //control(v, w, currW, &erro);
            if (abs((currW - w))>erro){
                erro = abs((currW - w));
            }
        }

        //faz um quadrado de re 
        for (int i = 0; i < 4; i++){
            previous_t = millis();
            while (t - previous_t <  700 ){
                t = millis();
                //rotina de controle anda de frente
                v = -0.2;
                w = 0;
                //readSpeeds(&currW);
                //control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }
            previous_t = millis();
            while (t - previous_t < 314 ){
                t = millis();
                //rotina de virar
                v = 0;
                w = -5;
                //readSpeeds(&currW);
                //control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }

            while (t - previous_t < 300 ){
                t = millis();
                //rotina de virar
                v = 0;
                w = 0;
                //readSpeeds(&currW);
                //control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }
        }  
        
        return erro;
    }

}