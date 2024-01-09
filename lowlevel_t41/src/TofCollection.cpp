#include "TofCollection.h"

TofCollection::TofCollection(DFRobot_MCP23017* ioExpander)
{
    this->ioExpander = ioExpander;
};

int TofCollection::init()
{
    // Determine I2C addresses to set
    fillAddr();

    // Set sensor values
    for (int i=0;i<L0X_NUM;++i)
    {
        l0xArr[i].setVars(addresses[i], pins[i], L0X_SAMPLING_TIME, ioExpander);
    }

    for (int i=0;i<L1X_NUM;++i)
    {
        l1xArr[i].setVars(addresses[L0X_NUM+i], pins[L0X_NUM+i], L1X_SAMPLING_TIME, ioExpander);
    }
    // Prepare sensors to set addresses
    resetSensors();
    disableSensors();

    int failNum {0};

    // Init all sensors with addresses
    for (int i=0;i<L0X_NUM;++i)
    {
        if (!l0xArr[i].init())
        {
            ++failNum;
        }
    }

    for (int i=0;i<L1X_NUM;++i)
    {
        if (!l1xArr[i].init())
        {
            ++failNum;
        }
    }

    return failNum;
}

void TofCollection::resetSensors()
{
    for (int i=0;i<L0X_NUM;++i)
    {
        l0xArr[i].reset();
        
    }

    for (int i=0;i<L1X_NUM;++i)
    {
        l1xArr[i].reset();
    }
}

void TofCollection::disableSensors()
{
    for (int i=0;i<L0X_NUM;++i)
    {
        l0xArr[i].disable();
    }

    for (int i=0;i<L1X_NUM;++i)
    {
        l1xArr[i].disable();
    }
}

void TofCollection::fillAddr()
{
    for (int i=0;i<TOF_NUM;++i)
    {
        addresses[i] = START_ADDR+i;
    }
}

bool TofCollection::update()
{
    int sensorsNotReady {0};
    for (int i=0;i<L0X_NUM;++i)
    {
        if (tofData[i].dataRdy == false)
        {
            int value = l0xArr[i].update();
            switch (value)
            {
                case -1:
                    ++sensorsNotReady;
                    break;
                default:
                    tofData[i].distance = value;
                    tofData[i].dataRdy = true;
                    break;
            }
        }
    }

    for (int i=0;i<L1X_NUM;++i)
    {
        if (tofData[L0X_NUM+1].dataRdy == false)
        {
            int value = l1xArr[i].update();
            switch (value)
            {
                case -1:
                    ++sensorsNotReady;
                    break;
                default:
                    tofData[L0X_NUM+i].distance = value;
                    tofData[L0X_NUM+i].dataRdy = true;
                    break;
            }
        }
    }

    // for (int i=0;i<TOF_NUM;++i)
    // {
    //     Serial.print(i);Serial.print(":");Serial.print(" dataRdy=");Serial.print(tofData[i].dataRdy);Serial.print("    distance=");Serial.print(distances[i]);
    // }
    // Serial.println("");
    // Serial.print("Sensors not ready: ");Serial.println(sensorsNotReady);
    

    if (sensorsNotReady != 0)
    {
        return false;
    }
    else
    {
        // Reset the data ready flags if all sensors were read correctly
        for (int i=0;i<L0X_NUM;++i)
        {
            tofData[i].dataRdy = false;
        }

        for (int i=0;i<L1X_NUM;++i)
        {
            tofData[L0X_NUM+i].dataRdy = false;
        }

        // Read the distance data into the distance array
        for (int i=0;i<TOF_NUM;++i)
        {
            distances[i] = tofData[i].distance;
        }

        return true;
    }

}

void TofCollection::test()
{

}