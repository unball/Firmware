#include <control.hpp>

namespace Control {

    using namespace Waves;

    // Converte de ticks/ms para m/s
    const double TICKS2METER = 2*PI*0.03*1000/(512*19);

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
        retorna por referência
    */
    void readSpeeds(Encoder::vel *enc, double *w){
        // Lê do encoders
        *enc = Encoder::encoder();

        // Lê do IMU
        Imu::imuAll imuData = Imu::imuRead();
        static double wcur = imuData.gyro.z;
        wcur = imuData.gyro.z * 0.8 + wcur * 0.2;

        *w = wcur;
    }

    /*
        Lê os encoders e a velocidade angular do IMU filtrada em primeira ordem e
        retorna a velocidade linear em m/s e a velocidade angular em rad/s por referência
    */
    void readSpeeds(double *v, double *w){
        Encoder::vel enc;
        double imuW;

        readSpeeds(&enc, &imuW);

        *v = linSpeed(enc);
        *w = angSpeed(imuW);
    }

    /*
        Implementa um PI digital com anti-windup para o motor A

        @param Recebe o erro
    */
    double PImotorA(double err){
        static double old_err;
        static double old_out;
        double out = (  1.6 * (err - 0.91  *  old_err) + old_out);
        old_err = err - (saturation(out)-out);
        
        old_out = out; //=  (abs(out) < 255)? out : 0;
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
        old_err = err - (saturation(out)-out);

        old_out = out; //(abs(out) < 255)? out : 0;;
        return out;
    }

    /*
        Implementa a malha de controle baixo nível

        @param v Velocidade linear de referência em m/s
        @param currV Velocidade linear medida por algum sensor (média dos encoders) em m/s
        @param w Velocidade angular de referência em rad/s
        @param currW Velocidade angular medida por algum sensor (IMU) em rad/s
    */
    void control(double v, double currV, double w, double currW){
        // Erro velocidade linear
        double eV = v - currV;

        // Erro velocidade angular
        double eW = w - currW;
        
        // Erro nos controladores
        double eA = eV + .3 * eW;
        double eB = eV - .3 * eW;

        // Controle digital
        int32_t controlA = (int32_t)PImotorA(eA/TICKS2METER);
        int32_t controlB = (int32_t)PImotorB(eB/TICKS2METER);


        // Passa para a planta a saída do controle digital e da malha acoplada
        Motor::move(0, deadzone(controlA, 7, -7));
        Motor::move(1, deadzone(controlB, 7, -7));
    }

    /*
        Lê velocidades do rádio, lê velocidades de referência e executa o controle
    */
    void stand_default(){

        // Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static Radio::vels velocidades;

        // Velocidades atuais medidas por sensores
        double currV, currW;
        
        // Lê velocidade do rádio
        Radio::receiveData(&velocidades);

        // Rádio foi perdido, mais de 2s sem mensagens
        if(Radio::isRadioLost()){
            velocidades.w = 0;
            velocidades.v = sine_wave();

            readSpeeds(&currV, &currW);
            control(velocidades.v, currV, velocidades.w, currW);
        }

        // Executa o controle normalmente com as velocidades de referência
        else {
            // Lê as velocidaes de sensores
            readSpeeds(&currV, &currW);

            // Executa a malha de controle
            control(velocidades.v, currV, velocidades.w, currW);

            // Zera a fila de recepção do rádio para ter sempre a mensagem mais recente
            Radio::radio.flush_rx();

            Led::red();
        }
    }

    /*
        Gera uma onda de teste para detectar deadzone, identificar ou validar
    */
    void stand_id(){
        // Velocidades de referência que serão compostas de acordo com a onda escolhida
        Radio::vels velocidades = {0};
        int16_t va=0,vb=0;

        // Buffer a ser transmitido pelo rádio
        static const size_t size = CONTROL_ID_BUFFER_SIZE;
        static Radio::reportStruct messages[size];
        static size_t index = 0;

        // Dados dos sensores encoders e imu
        Encoder::vel enc;
        double imuW;

        // Lê os dados dos sensores
        readSpeeds(&enc, &imuW);

        // Escolhe quais serão as entradas da planta
        #if   (CONTROL_ID_MODE == CONTROL_ID_MODE_DEADZONE)
            triangular_wave(&va, &vb, 64, 500);

            // Alimenta a planta com as entradas
            Motor::move(0, va);
            Motor::move(1, vb);

        #elif (CONTROL_ID_MODE == CONTROL_ID_MODE_ID)
            square_wave(&va, &vb, 20, 750);

            // Alimenta a planta com as entradas
            Motor::move(0, deadzone(va, 7, -7));
            Motor::move(1, deadzone(vb, 7, -6));

        #elif (CONTROL_ID_MODE == CONTROL_ID_MODE_VALIDATION)
            velocidades.v = square_wave(1.2, 400);
            velocidades.w = 0;

            control(velocidades.v, velocidades.w, linSpeed(enc), angSpeed(imuW));

        #endif

        // Compõe a mensagem a ser enviada pelo rádio
        Radio::reportStruct message = {
            .time = micros(),
            .v = velocidades.v, 
            .w = velocidades.w, 
            .va = va,
            .vb = vb,
            .enca = enc.motorA, 
            .encb = enc.motorB,
            .imuw = (float)imuW
        };
        messages[index] = message;

        #if   (CONTROL_ID_TRANSFER == CONTROL_ID_TRANSFER_SERIAL)

            // Reporta mensagem via serial
            Serial.printf("%d %lf %lf %d %d %lf %lf %lf\n", message.time, message.v, message.w, va, vb, message.enca, message.encb, message.imuw);

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
    }

    /*
        Executa um loop do controle usando como referência uma mensagem do rádio.
        A função pode mudar de comportamento se a definição CONTROL_ID for verdadeira,
        neste caso o comportamento é de fazer um loop de validação, identificação ou de detecção da deadzone.
    */
    void stand() {
        
        #if CONTROL_ID // Loop se estiver no modo de identificação
            stand_id();
        #else // Loop padrão
            stand_default();
        #endif
    }

}//end namespace