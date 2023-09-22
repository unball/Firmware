#include <control.hpp>

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
    double PImotorA(double err){
        static double old_err;
        static double old_out;
        double out = (  1.6 * (err - 0.91  *  old_err) + old_out);
        old_err = err; //- (saturation(out)-out);
        //  1.6 * (1-0.91 * z^-1)/(1-z^-1)
        //  1.6 * (z-0.91)/(z-1)    Projetado a partir do LGR pro motor (identificação o motor)
        //
        // 1 - Identificar novo motor (com carga/com roda e no chão)
        //                              -MATLAB identificação de sistemas
        //                              -sintonização de constantes
        // 2 - Projetar um controlador (contínuo)
        // 3 - Discretizar controlador (mapeamento direto com transformada z)
        //                              -comando MATLAB: c2d(tf,T,"matched")
        // 4 - Expandir função de transferencia e isolar U(z)
        // 5 - Transformada inversa de z
        //
        //  z = e^(sT)

        
        old_out = (abs(out) < 255)? out : 0;    // anti-windup (evita que o erro de saturação seja considerado como erro)
        return out;
    }


    /*
        Implementa um PI digital com anti-windup para o motor B

        @param Recebe o erro
    */
    double PImotorB(double err){
        static double old_err;
        static double old_out;
        double out =  (  1.6 * (err  - 0.91 *  old_err) + old_out);
        old_err = err; //- (saturation(out)-out);

        old_out = (abs(out) < 255)? out : 0;;
        return out;
    }

    /*
        Implementa a malha de controle baixo nível

        @param v Velocidade linear de referência em m/s
        @param currV Velocidade linear medida por algum sensor (média dos encoders) em m/s
        @param w Velocidade angular de referência em rad/s
        @param currW Velocidade angular medida por algum sensor (IMU) em rad/s
    */
    void control(double w, double currW){
        
        if (w == 0){
            Motor::stop();
            return;
        }

        // Erro velocidade angular
        double eW = w - currW;
        
        // Erro nos controladores
        // double eA = eV + .2 * eW;
        // double eB = eV - .2 * eW;

        // // Controle digital
        // int32_t controlA = (int32_t)PImotorA(eA/TICKS2METER);
        // int32_t controlB = (int32_t)PImotorB(eB/TICKS2METER);


        // // Passa para a planta a saída do controle digital e da malha acoplada
        // Motor::move(0, deadzone(controlA, 7, -7));
        // Motor::move(1, deadzone(controlB, 7, -7));
    }

    /*
        Lê velocidades do rádio, lê velocidades de referência e executa o controle
    */
    void stand(){

        // Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static double w = 0;

        // Velocidades atuais medidas por sensores
        double currW;
        
        // Lê velocidade do rádio
        // Radio::receiveData(&v, &w);
        

        // Rádio foi perdido, mais de 2s sem mensagens
        // if(Radio::isRadioLost()){
        //     w = 0;
        //     v = sine_wave();

        //     readSpeeds(&currV, &currW);
        //     control(v, currV, w, currW);
        // }

        // Executa o controle normalmente com as velocidades de referência
        // else {
            // Lê as velocidaes de sensores
            readSpeeds(&currW);

            // Executa a malha de controle
            control(w, currW);
        // }
    }

}//end namespace