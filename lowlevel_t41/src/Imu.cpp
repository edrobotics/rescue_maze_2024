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


void Imu::printVals()
{
    Serial.print("IMU: ");

    Serial.print("real: ");Serial.print(rotationVector.floats[Quaternion::term_real], 5);Serial.print("  ");
    Serial.print("i: ");Serial.print(rotationVector.floats[Quaternion::term_i], 5);Serial.print("  ");
    Serial.print("j: ");Serial.print(rotationVector.floats[Quaternion::term_j], 5);Serial.print("  ");
    Serial.print("k: ");Serial.print(rotationVector.floats[Quaternion::term_k], 5);Serial.print("  ");
    Serial.println("");
}