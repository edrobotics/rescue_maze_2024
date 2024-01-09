#include "TofVl53l1x.h"


TofVl53l1x::TofVl53l1x()
{

}

void TofVl53l1x::setVars(int addr, DFRobot_MCP23017::ePin_t pin, int samplingTime, DFRobot_MCP23017* ioExpander)
{
    i2cAddr = addr;
    this->ioExpander = ioExpander;
    xShutPin = pin;
    this->samplingTime = samplingTime;
}

bool TofVl53l1x::init()
{
    enable();
    delay(10);

    sensor.setTimeout(TIMEOUT_TIME);
    if (!sensor.init())
    {
        Serial.println("[VL53L1X] Initialization failed");
        return false;
    }

    // Address setting
    sensor.setAddress(i2cAddr);

    // Parameter setting
    sensor.setDistanceMode(VL53L1X::Short); // Short distance mode
    sensor.setMeasurementTimingBudget(TIMING_BUDGET); // Make it quick

    sensor.startContinuous(samplingTime);

    // Done
    return true;

}


int TofVl53l1x::update()
{
    // Prevent reading if data is not ready
    if (sensor.dataReady()==false)
    {
        return -1;
    }
    // Data is ready and we can now read
    int retVal = sensor.read(false);
    if (sensor.timeoutOccurred())
    {
        Serial.println("[VL53L1X] TIMEOUT");
    }
    return retVal;
}

void TofVl53l1x::reset()
{
    disable();
    delay(10);
    enable();
}

void TofVl53l1x::disable()
{
    ioExpander->pinMode(xShutPin, OUTPUT);
    ioExpander->digitalWrite(xShutPin, LOW);
}

void TofVl53l1x::enable()
{
    ioExpander->pinMode(xShutPin, INPUT);
}