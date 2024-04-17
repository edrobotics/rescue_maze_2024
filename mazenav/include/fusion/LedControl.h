#pragma once

#include <thread>
#include <chrono>
#include <iostream>

#include "fusion/PiAbstractor.h"

class LedControl
{
    public:
        LedControl();
        // Perform initialization
        void init(PiAbstractor* piAbs);
        // Blinks the LEDs for 5 seconds
        void blinkLedVictimFound();
        void waitForFinish();

        void test();

    private:
        PiAbstractor* piAbs {};
        double blinkFreq {1};

        int led1Pin {25};
        int led2Pin {28};

        void blinkLeds(int cycles, double freq);
        void turnLightsOn();
        void turnLightsOff();

        std::thread blinkThread{};

};