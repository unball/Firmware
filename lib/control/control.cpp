#include <control.hpp>

namespace Control {

    using namespace Waves;

    double erro = 0;
    double err_sum = 0;
    double last_err = 0;

    double kp = 1.08;
    double ki = 0.15;
    double kd = -0.03;

    /*
        Controle dos robôs
         - Allia Massouh - Robô 0
        Dz = 10   /  Dz = 32
        Kp = 1.62 /  Kp = 0.54
        Ki = 0.05 /  Ki = 0.05
        Kd = -0.08/  Kd = -0.03
        
         - Tatima Cetando - Robô 1
        Dz = 5   /  Dz = 32
        Kp = 2.05 /  Kp = 0.54
        Ki = 0.05 /  Ki = 0.10
        Kd = -0.13/  Kd = -0.08

         - Paula Traz - Robô 2
        Dz = 10   /  Dz = 02
        Kp = 1.62 /  Kp = 0.74
        Ki = 0.05 /  Ki = 0.05
        Kd = -0.15/  Kd = -0.13

         - Ondrej Kudela
        Dz = ??? /  Dz = 32
        Kp = ??? /  Kp = ???
        Ki = ??? /  Ki = ???
        Kd = ??? /  Kd = ???

    */
    

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

    inline double angvel2PWM(double value){
        return value*pwm_max/(v_max / r);
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
        
    }
    
    
    void stand(){

        // Velocities to be read by Wi-Fi, they are static in case Wifi::receiveData does not receive anything, it keeps the previous velocity
        static double v = 0;
        static double w = 0;
        float v_int = 0;
        float w_int = 0;
       

        // Velocidades atuais medidas por sensores
        double currW;
        
        // Lê velocidades pelo Wifi
        Wifi::receiveData(&v_int, &w_int);

        v = static_cast<double>(v_int);
        w  = static_cast<double>(w_int);

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
            //speed2motors(2, 65);
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
                readSpeeds(&currW);
                control(v, w, currW, &erro);
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
                readSpeeds(&currW);
                control(v, w, currW, &erro);
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
            readSpeeds(&currW);
            control(v, w, currW, &erro);
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
                v = -0.02;
                w = 0;
                readSpeeds(&currW);
                control(v, w, currW, &erro);
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
                readSpeeds(&currW);
                control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }

            while (t - previous_t < 300 ){
                t = millis();
                //rotina de virar
                v = 0;
                w = 0;
                readSpeeds(&currW);
                control(v, w, currW, &erro);
                if (abs((currW - w))>erro){
                    erro = abs((currW - w));
                }
            }
        }  
        
        return erro;
    }

    void twiddle(){
        double target;
        double k[3];
        double dk[3];
        double ksi = 0.3;
        dk[0] = 0.538265;
        dk[1] = 0.049981750000000005; 
        dk[2] = 0.049981750000000005;
        k[0] = kp;
        k[1] = ki;
        k[2] = kd;
        target = test();
        for (int i = 0; i < 3; i++){
            k[i] += dk[i];
            kp = k[0];
            ki = k[1];
            kd = k[2];
            erro = test();
            
            if (erro < target){
                target = erro;
                dk[i] *= 1+ksi; 
            }
            else{
                k[i] -= 2*dk[i];

                kp = k[0];
                ki = k[1];
                kd = k[2];
                erro = test();

                if (erro < target){
                    target = erro;
                    dk[i] *= 1+ksi;
                }
                else{
                    k[i]  = dk[i];
                    dk[i] *= 1 - ksi;
                }
            }

        }

        while (true){
            control(0, 0, 0, &erro);
            Serial.print("kp: ");
            Serial.println(kp);
            Serial.print("ki: ");
            Serial.println(ki);
            Serial.print("kd: ");
            Serial.println(kd);
            
            
            
        }
             
    }

    void deadzone_tester(){
        int v = 2;
        Motor::move(0, v);
        Motor::move(1, v);
    }
        

}