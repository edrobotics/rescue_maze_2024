#include "fusion/PiAbstractor.h"



void PiAbstractor::init()
{
    std::lock_guard<std::mutex> lock(mtx_general);
    #ifdef ENV_PI
    wiringPiSetup();
    #endif
}

void PiAbstractor::pinModeTS(int pin, PinMode mode)
{
    std::lock_guard<std::mutex> lock(mtx_general);
    #ifdef ENV_PI
    if (mode==PinMode::inputPullup)
    {
        pinMode(pin, INPUT);
        pullUpDnControl(pin, PUD_UP);
    }
    else if (mode==PinMode::inputPulldown)
    {
        pinMode(pin, INPUT);
        pullUpDnControl(pin, PUD_DOWN);
    }
    else if (mode==PinMode::output)
    {
        pinMode(pin, OUTPUT);
    }
    #endif

}

void PiAbstractor::digitalWriteTS(int pin, bool state)
{
    std::lock_guard<std::mutex> lock(mtx_general);
    #ifdef ENV_PI
    digitalWrite(pin, state);
    #endif

}

bool PiAbstractor::digitalReadTS(int pin)
{
    std::lock_guard<std::mutex> lock(mtx_general);
    #ifdef ENV_PI
    return digitalRead(pin);
    #else
    return false;
    #endif

}