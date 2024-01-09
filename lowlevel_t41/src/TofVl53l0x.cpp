#include "TofVl53l0x.h"


TofVl53l0x::TofVl53l0x()
{

}

void TofVl53l0x::setVars(int addr, DFRobot_MCP23017::ePin_t pin, int samplingTime, DFRobot_MCP23017* ioExpander)
{
    i2cAddr = addr;
    this->ioExpander = ioExpander;
    xShutPin = pin;
    this->samplingTime = samplingTime;
}

bool TofVl53l0x::init()
{
    enable();
    delay(10);

    sensor.setTimeout(TIMEOUT_TIME);
    if (!sensor.init())
    {
        Serial.println("[VL53L0X] Initialization failed");
        return false;
    }

    // Address setting
    sensor.setAddress(i2cAddr);

    // Parameter setting
    // Configure to prefer speed over accuracy
    sensor.setMeasurementTimingBudget(TIMING_BUDGET);

    sensor.startContinuous(samplingTime);

    // Done
    return true;

}


int TofVl53l0x::update()
{
    // Prevent reading if data is not ready
    if (sensor.dataReady()==false)
    {
        return -1;
    }
    // Data is ready and we can now read
    int retVal = sensor.readNonBlock();
    if (sensor.timeoutOccurred())
    {
        Serial.println("[VL53L0X] TIMEOUT");
    }
    return retVal;
}

void TofVl53l0x::reset()
{
    disable();
    delay(10);
    enable();
}

void TofVl53l0x::disable()
{
    ioExpander->pinMode(xShutPin, OUTPUT);
    ioExpander->digitalWrite(xShutPin, LOW);
}

void TofVl53l0x::enable()
{
    ioExpander->pinMode(xShutPin, INPUT);
}