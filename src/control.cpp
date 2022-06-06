#include <control.hpp>

namespace Control {

    using namespace Waves;

    /*
        Executa um loop do controle usando como referência uma mensagem do rádio.
    */
   
    void stand() {
        // Velocidades a serem lidas do rádio, são estáticas de modo que se Radio::receiveData não receber nada, mantém-se a velocidade anterior
        static double v = 0;
        static double w = 0;
        
        // Lê velocidade do rádio
        Radio::receiveData(&v, &w);
        
        // Rádio foi perdido, mais de 2s sem mensagens
        if(Radio::isRadioLost()){
            w = 0;
            v = sine_wave();

            Motor::move(0, v);
            Motor::move(1, v);
        }

        // Executa o controle normalmente com as velocidades de referência
        else {
            // Passa para a planta a saída do controle digital e da malha acoplada
            Motor::move(0, v);
            Motor::move(1, w);
        }
    }

}//end namespace