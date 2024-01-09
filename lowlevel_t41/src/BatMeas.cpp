#include "BatMeas.h"


BatMeas::BatMeas(int pin, int r1, int r2)
{
    this->measPin = pin;
    this->R1 = r1;
    this->R2 = r2;
    pinMode(measPin, INPUT);
}


float BatMeas::getVoltage()
{
    float scaled = static_cast<float>(analogRead(measPin))*3.3/static_cast<float>(1023);
    float actual = scaled*(R1+R2)/R2;
    return actual;
    // return analogRead(measPin);
    
}