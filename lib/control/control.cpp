#include <control.hpp>
#include <encoder.hpp>

namespace Control {

    using namespace Waves;

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
        return min(max(vin, -pwm_max), pwm_max);
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

    inline double angvel2PWM(double value){
        return value*pwm_max/(v_max/r);
    }

    void matrix_multiply(double mat[2][2], double vec[2], double result[2]) {
        for (int i = 0; i < 2; i++) {
            result[i] = 0;
            for (int j = 0; j < 2; j++) {
                result[i] += mat[i][j] * vec[j];
            }
        }
    }

    void vector_add(double vec1[2], double vec2[2], double result[2]) {
        for (int i = 0; i < 2; i++) {
            result[i] = vec1[i] + vec2[i];
        }
    }

    /*
        Implementa um PI digital com anti-windup para o motor A

        @param Recebe o erro
    */

    Control_cee CEE(double v, double eV, double w, double eW){

        Control_cee y;

        

        double e[2] = {eV, eW};  // Vetor de erros
        double x[2] = {v, w};    // Vetor de estados
        double u[2];             // Vetor de entrada


        //Matriz de ganho
        double K[2][2] = {{Kv, 0},
                          {0, Kw}};

        //calcula a entrada de controle
        matrix_multiply(K, e, u);

        //Matrizes de estados
        double A[2][2] = {{1, 0},
                          {0, 1}};

        double B[2][2] = {{1, 0},
                          {0, 1}};

        double C[2][2] = {{1, 0},
                          {0, 1}};

        double D[2][2] = {{0, 0},
                          {0, 0}};

        double aux[2];
        double aux2[2];
        double new_x[2];

        // Calcular new_x = A*x + B*u
        matrix_multiply(A, x, aux2);
        matrix_multiply(B, u, aux);
        vector_add(aux, aux2, new_x);

        // Calcular a saída y = C*new_x + D*u
        matrix_multiply(C, new_x, aux2);
        matrix_multiply(D, u, aux);
        vector_add(aux, aux2, new_x);       
        
        
        y.v = new_x[0];
        y.w = new_x[1];


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
        vr = angvel2PWM(vr);
        vl = angvel2PWM(vl);
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
        
        saida_cee = CEE(v,eV, w, eW);
        
        speed2motors(saida_cee.v, saida_cee.w);

    }


    void stand(){

        // Velocities to be read by Wi-Fi, they are static in case Wifi::receiveData does not receive anything, it keeps the previous velocity
        static double v;
        static double w;
        static int16_t v_int;
        static int16_t w_int;
       

        // Velocidades atuais medidas por sensores
        double currW;
        double currV;
        
        // Lê velocidades pelo Wifi
        Wifi::receiveData(&v_int, &w_int);

        //demutiplexa velocidades
        v = ((float)v_int) * 2.0 / 32767;
        w = ((float)w_int) * 64.0 / 32767;

        if(false){           
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

}