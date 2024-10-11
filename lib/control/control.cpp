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
         
        *v = Encoder::calibration_encoder(linSpeed(Encoder::encoder()));
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
        return value*pwm_max/(v_max + (L/2)*w_max) / r;
    }

    double err_sum_w = 0; // Erro integral para PID em w
    double last_err_w = 0; // Último erro para PID em w

    void matrix_multiply(double mat[4][4], double vec[4], double result[4]) {
        for (int i = 0; i < 4; i++) {
            result[i] = 0;
            for (int j = 0; j < 4; j++) {
                result[i] += mat[i][j] * vec[j];
            }
        }
    }

    void vector_add(double vec1[4], double vec2[4], double result[4]) {
        for (int i = 0; i < 4; i++) {
            result[i] = vec1[i] + vec2[i];
        }
    }

    Control_cee CEE(double v, double eV, double w, double eW) {
        Control_cee y;
        
        double kp_v = 1.08;
        double kp_w = 1.08;
        double ki_w = 0.15;
        double kd_w = -0.3;

        // Vetor de estados
        double x[4] = {v, w, err_sum_w, last_err_w};

        // Matrizes do espaço de estados
        double A[4][4] = {
            {1, 0, 0, 0}, // Dinâmica de v
            {0, 1, 0, 0}, // Dinâmica de w
            {0, 0, 1, 0}, // Dinâmica do erro integral w
            {0, 0, 0, 1}  // Dinâmica da derivada do erro w
        };

        double B[4][2] = {
            {1, 0},
            {0, 1},
            {0, 1},
            {0, 0}
        };

        double C[2][4] = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };

        double D[2][2] = {
            {0, 0},
            {0, 0} 
        };

        // Atualiza os erros
        err_sum_w += eW;
        double eP_v = eV;
        double eP_w = eW;
        double eD_w = eW - last_err_w;

        //filtro anti-windup
        err_sum_w = (abs(err_sum_w) < 64.0)? err_sum_w : 0;

        double u[2];
        u[0] = kp_v * eP_v;
        u[1] = kp_w * eP_w + ki_w * err_sum_w + kd_w * eD_w;

        double new_x[4];
        double aux1[4];

        // x' = Ax + Bu
        matrix_multiply(A, x, aux1);
        double temp_u[2] = {u[0], u[1]};
        double Bu[4] = {0, 0, 0, 0};

        Bu[1] = u[1];
        vector_add(aux1, Bu, new_x);

        // Atualizar o vetor de estados
        x[0] = new_x[0]; 
        x[1] = new_x[1]; 
        x[2] = err_sum_w;
        last_err_w = eW;

        y.v = C[0][0] * x[0] + C[0][1] * x[1] + C[0][2] * x[2] + C[0][3] * x[3];
        y.w = C[1][0] * x[0] + C[1][1] * x[1] + C[1][2] * x[2] + C[1][3] * x[3];
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

        //Se o controle no espaço de estados estiver ligado considera o eV
        if (CEE_ON){
            saida_cee = CEE(v,eV, w, eW);
        }
        else{
            saida_cee = CEE(v,0, w, eW);
        }
                
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
        v =  ((float)v_int) * 2.0 / 32767;
        w  = ((float)w_int) * 64.0 / 32767;

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