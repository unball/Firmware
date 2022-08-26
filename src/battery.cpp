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
        3.3V 1023
        2.96V x
    */

    void measure(float *voltage){
        // read the input on analog pin 0:
        int sensorValue = analogRead(BATT);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
        float voltage_read = sensorValue * (2.96 / 917.6);
        *voltage = voltage_read * (7.4 / 2.96);

        Serial.print("A0 CONTÉM[");Serial.print(sensorValue);Serial.println("]");
        Serial.print("voltagem:[");Serial.print(voltage_read);Serial.println("]V");
        Serial.println("#####");
        //*voltage = map_float(readValue, 0, 1023, 0, 100);
    }

}