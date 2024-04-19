#include "fusion/ColourSensor.h"


ColourSensor::ColourSensor(i2cCommunicator* i2cComm)
: colSens {TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X, i2cComm}
{
}


void ColourSensor::init()
{
    while(!colSens.begin())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        std::cout << "TCS34725 not found\n";
    }
}


ColourSample ColourSensor::getValuesFromSensor()
{
    ColourSample retVal {};
    long milsSinceRead {std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastReadTime).count()};
    if (!(milsSinceRead>INTEGRATION_TIME_MS))
    {
        retVal.wasDone = false;
        return retVal;
    }

    uint16_t r {};
    uint16_t g {};
    uint16_t b {};
    uint16_t c {};
    colSens.getRawData(&r, &g, &b, &c);

    retVal.values.at(fcol_r) = r;
    retVal.values.at(fcol_g) = g;
    retVal.values.at(fcol_b) = b;
    retVal.values.at(fcol_c) = c;
    retVal.wasDone = true;
    
    lastReadTime = std::chrono::steady_clock::now();

    return retVal;
}


bool ColourSensor::updateVals()
{
    colSample = getValuesFromSensor();
    return colSample.wasDone;
}