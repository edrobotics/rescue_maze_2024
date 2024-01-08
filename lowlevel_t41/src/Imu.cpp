#include <Imu.h>

Imu::Imu()
{

}

bool Imu::init()
{
    bool retVal = true;

    if (!bno085.init())
    {
        retVal = false;
    }

    // Init other IMUs

    return retVal;
}


bool Imu::runLoop()
{
    bool retVal = true;

    if (bno085.runLoop())
    {
        rotationVector = bno085.rotationVector;
    }
    else
    {
        retVal = false;
    }

    return retVal;
}
