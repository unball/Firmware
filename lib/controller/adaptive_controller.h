#ifndef ADAPTIVE_CONTROLLER_H
#define ADAPTIVE_CONTROLLER_H

#include <Arduino.h>

namespace AdaptiveController {
    void update();
    void bypassControl();

    void setReferences(float omega_L_ref, float omega_R_ref);

    float getTheta1Left();
    float getTheta2Left();
    float getTheta1Right();
    float getTheta2Right();

    float getOmegaLeft();
    float getOmegaRight();

    float getControlSignalLeft();
    float getControlSignalRight();
}

#endif // ADAPTIVE_CONTROLLER_H
