#include <control.hpp>

namespace Control {

    using namespace Waves;

    double erro = 0;
    double err_sum = 0;
    double last_err = 0;

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
        Função que recebe velocidade angular do IMU em º/ms e converte para velocidade angular em rad/s
    */
    inline double angSpeed(double imuW){
        return imuW * PI / 180;
    }

    /*
        Lê os encoders e a velocidade angular do IMU filtrada em primeira ordem e
        retorna a velocidade linear em m/s e a velocidade angular em rad/s por referência
    */
    void readSpeeds(double *w){
        // *w = angSpeed(-IMU::get_w());
        *w = -IMU::get_w();
    }

    /*
        Implementa um PI digital com anti-windup para o motor A

        @param Recebe o erro
    */
    double PID(double v, double err){
        err_sum += err;
        err_sum = (abs(err_sum) < 64.0)? err_sum : 0;
        
        double P = err * kp;
        double I = err_sum * ki;
        double D = (err - last_err) * kd;
        // D = 0.0;
        
        double output = P+I+D;
        last_err = err;

	    return output;
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

        Serial.println("##########################################");
        Serial.println("""""""""""""""""'controlR'""""""""""""""""");
        Serial.println(vr);
        Serial.println("""""""""""""""""'controlLL'""""""""""""""""");
        Serial.println(vl);
        Serial.println("""""""""""""""""'w'""""""""""""""""");
        Serial.println(w);
        Serial.println("""""""""""""""""'v'""""""""""""""""");
        Serial.println(v);
        Serial.println("##########################################");
    }

    /*
        Implementa a malha de controle baixo nível

        @param v Velocidade linear de referência em m/s
        @param w Velocidade angular de referência em rad/s
        @param currW Velocidade angular medida por algum sensor (IMU) em rad/s
        @param *erro ponteiro para a variavel global de erro
    */
    void control(double v, double w, double currW, double *erro){
        //TODO: deadzone?
        if (v == 0 && w == 0){
            Motor::stop();
            return;
        }

        // Angular velocity error
        double eW = w - currW;
        

        w = PID(v, eW);

        speed2motors(v,w);

        /*if (v > 0 ) v = map(v, 0, 255, 60, 255);
        if (v < 0 ) v = map(v, 0, -255, -60, -255);

        int32_t controlR = (int32_t)saturation((v - w));
        int32_t controlL = (int32_t)saturation((v + w));

        controlR = (int32_t)saturation((deadzone(controlR, motor_deadzone, -motor_deadzone)));
        controlL = (int32_t)saturation((deadzone(controlL, motor_deadzone, -motor_deadzone)));

        if (controlR < 15 && controlR > -15) controlR = 0;
        if (controlL < 15 && controlL > -15) controlL = 0;

        // Passes the control output to the plant 
        // Motor::move(0, deadzone(controlR, 7, -7));
        // Motor::move(1, deadzone(controlL, 7, -7));
        Motor::move(0, controlR);
        Motor::move(1, controlL);
        Serial.println("##########################################");
        Serial.println("""""""""""""""""'controlR'""""""""""""""""");
        Serial.println(controlR);
        Serial.println("""""""""""""""""'controlLL'""""""""""""""""");
        Serial.println(controlL);
        Serial.println("""""""""""""""""'w'""""""""""""""""");
        Serial.println(w);
        Serial.println("""""""""""""""""'v'""""""""""""""""");
        Serial.println(v);
        Serial.println("##########################################");

        *erro = eW;*/
        
    }
    
    
    void stand(){

        // Velocities to be read by Wi-Fi, they are static in case Wifi::receiveData does not receive anything, it keeps the previous velocity
        static double v = 0; //vl
        static double w = 0; //vr

        // Velocidades atuais medidas por sensores
        double currW;
        
        // Lê velocidades pelo Wifi
        Wifi::receiveData(&v, &w);

        if(Wifi::isCommunicationLost()){
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
            readSpeeds(&currW);
            // Execute the control loop
            control(v, w, currW, &erro);
        }
    }

}