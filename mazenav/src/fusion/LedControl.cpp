#include "fusion/LedControl.h"


LedControl::LedControl()
{
}

void LedControl::init()
{

    #ifdef ENV_PI
    wiringPiSetup();
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    #endif
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
    #ifdef ENV_PI
    digitalWrite(led1Pin, 0);
    digitalWrite(led2Pin, 0);
    #endif
}

void LedControl::turnLightsOn()
{
    #ifdef ENV_PI
    std::cout << "Turning lights on\n";
    digitalWrite(led1Pin, 1);
    digitalWrite(led2Pin, 1);
    #endif
}