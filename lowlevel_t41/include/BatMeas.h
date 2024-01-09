#pragma once
#include <Arduino.h>

class BatMeas
{
    public:
        // pin - pin connected to voltage divider
        // r2 - the resistor that is measured over
        // r1 - the other resistor
        BatMeas(int pin, int r1, int r2);

        float getVoltage();

    private:
        int measPin {};
        int R1 {};
        int R2 {};
};