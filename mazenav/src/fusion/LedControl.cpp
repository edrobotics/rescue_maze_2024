#include "fusion/LedControl.h"


LedControl::LedControl()
{
}

void LedControl::init(PiAbstractor* piAbs)
{
    this->piAbs = piAbs;
    piAbs->pinModeTS(led1Pin, PiAbstractor::PinMode::output);
    piAbs->pinModeTS(led2Pin, PiAbstractor::PinMode::output);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void LedControl::blinkLedVictimFound()
{
    blinkThread = std::thread{&LedControl::blinkLeds, this, 5, blinkFreq};
}

void LedControl::waitForFinish()
{
    blinkThread.join();
}

void LedControl::test()
{
    turnLightsOn();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    turnLightsOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    turnLightsOn();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    turnLightsOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


void LedControl::blinkLeds(int cycles, double freq)
{
    int timeMillis {static_cast<int>(1000.0/(2*freq))};
    turnLightsOn();
    for (int i=0;i<cycles-1;++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeMillis));
        turnLightsOff();
        std::this_thread::sleep_for(std::chrono::milliseconds(timeMillis));
        turnLightsOn();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(timeMillis));
    turnLightsOff();
}


void LedControl::turnLightsOff()
{
    piAbs->digitalWriteTS(led1Pin, 0);
    piAbs->digitalWriteTS(led2Pin, 0);
}

void LedControl::turnLightsOn()
{
    piAbs->digitalWriteTS(led1Pin, 1);
    piAbs->digitalWriteTS(led2Pin, 1);
}