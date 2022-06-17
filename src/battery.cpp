#include <battery.hpp>

namespace Battery{

    float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
        const float dividend = out_max - out_min;
        const float divisor = in_max - in_min;
        const float delta = x - in_min;

        return (delta * dividend + (divisor / 2)) / divisor + out_min;
    }

    /*
        Lê a entrada analógica A0 e retorna a tensão na bateria
    */

    void measure(float *voltage){
        int readValue;

        readValue = analogRead(BATT);
        *voltage = map_float(readValue, 0, 1023, 0, 100);
    }

}