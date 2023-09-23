#include <control.hpp>

namespace Control {

    using namespace Waves;

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
        *w = angSpeed(IMU::get_w());
    }

    /*
        Implementa um PI digital com anti-windup para o motor A

        @param Recebe o erro
    */
    double PID(double v, double err){
        err_sum += err;
        
        double P = err * kp;
        double I = err_sum * ki;
        double D = (err - last_err) * kd;
        
        double output = P+I+D;

	    return output;
    }

    /*
        Implementa a malha de controle baixo nível

        @param v Velocidade linear de referência em m/s
        @param w Velocidade angular de referência em rad/s
        @param currW Velocidade angular medida por algum sensor (IMU) em rad/s
    */
    void control(double v, double w, double currW){
        
        if (v == 0 && w == 0){
            Motor::stop();
            return;
        }

        // Angular velocity error
        double eW = w - currW;

        w = PID(v, eW);

        int32_t controlR = (int32_t)(v - w);
        int32_t controlL = (int32_t)(v + w);

        // // Passa para a planta a saída do controle digital e da malha acoplada
        Motor::move(0, deadzone(controlR, 7, -7));
        Motor::move(1, deadzone(controlL, 7, -7));
    }

    /*
        Lê velocidades do rádio, lê velocidades de referência e executa o controle
    */
    void stand(){

        // Velocities to be read by Wi-Fi, they are static in case Wifi::receiveData does not receive anything, it keeps the previous velocity
        static double v = 0;
        static double w = 0;

        // Velocidades atuais medidas por sensores
        double currW;
        
        // Lê velocidades pelo Wifi
        Wifi::receiveData(&v, &w);

        if(Wifi::isCommunicationLost()){
			w = 0;
			v = Waves::sine_wave();

            readSpeeds(&currW);
            control(v, w, currW);

			Motor::move(0, v);
			Motor::move(1, v);
		}

        // Execute the control normally with the reference velocities
        // Read the velocities through the sensor
        readSpeeds(&currW);

        // Execute the control loop
        control(v , w, currW);
    }

}