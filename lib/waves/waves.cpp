#include <waves.hpp>

namespace Waves{
    
    /*
        Implementa uma onda quadrada

        @param amplitude Valor em m/s da amplitude da onda
        @param maxCount Número de chamadas a função necessário para realizar um quarto do ciclo da onda
    */
    double square_wave(double amplitude, uint32_t maxCount){
        static uint32_t square_wave_cont;
        static int8_t state = 3, out;
        if(square_wave_cont > maxCount){
            square_wave_cont = 0;
            state = (state+1)%4;
        }
        out = (state % 2) * (state-2);
        square_wave_cont++;
        return out*amplitude;
    }

    /*
        Implementa uma onda senoidal
    */
    float sine_wave(){
        static int32_t sine_wave_cont;
        if(sine_wave_cont >= 200000){
            sine_wave_cont = 0;
        }
        sine_wave_cont ++;
        return 25*sin(sine_wave_cont/400.0);
    }
    
    /*
        Implementa uma onda triangular

        @param v1 Ponteiro para a velocidade do motor A
        @param v2 Ponteiro para a velocidade do motor B
        @param amplitude Valor em ticks/ms da amplitude da onda
        @param maxCount Número de chamadas a função necessário para realizar um quarto do ciclo da onda
    */
    void triangular_wave(int16_t *v1, int16_t *v2, int32_t amplitude, int32_t maxCount){
        static int32_t triangular_wave_cont;
        static int8_t state = 3, out;
        if(abs(triangular_wave_cont) >= maxCount){
            triangular_wave_cont = 0;
            state = (state+1)%4;
        }
        triangular_wave_cont += 1;
        out = (state % 2) * (state-2);
        *v1 = out*triangular_wave_cont*amplitude/maxCount;
        *v2 = out*triangular_wave_cont*amplitude/maxCount;
    }

    /*
        Implementa uma onda senoidal

        @param v1 Ponteiro para a velocidade do motor A
        @param v2 Ponteiro para a velocidade do motor B
    */
    void sine_wave(int16_t *v1, int16_t *v2){
        static int32_t sine_wave_cont;
        if(sine_wave_cont >= 200000){
            sine_wave_cont = 0;
        }
        sine_wave_cont ++;
        *v1 = 25*sin(sine_wave_cont/400.0);
        *v2 = 25*sin(sine_wave_cont/400.0);
    }

    /*
        Implementa uma onda quadrada

        @param v1 Ponteiro para a velocidade do motor A
        @param v2 Ponteiro para a velocidade do motor B
        @param amplitude Valor em ticks/ms da amplitude da onda
        @param maxCount Número de chamadas a função necessário para realizar um quarto do ciclo da onda
    */
    void square_wave(int16_t *v1, int16_t *v2, int32_t amplitude, uint32_t maxCount){
        static uint32_t square_wave_cont;
        static int8_t state = 3, out;
        if(square_wave_cont > maxCount){
            square_wave_cont = 0;
            state = (state+1)%4;
        }
        out = (state % 2) * (state-2);
        *v1 = out*amplitude;
        *v2 = out*amplitude;
        square_wave_cont++;
    }


    void step(int16_t *v1, int16_t *v2){
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

}