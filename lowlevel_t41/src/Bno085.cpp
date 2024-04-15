#include "Bno085.h"

Bno085::Bno085(int address)
{
    i2cAddr = address;
}

bool Bno085::init()
{
    bool retVal = true;
    if (!bno085.begin_I2C(i2cAddr))
    {
        Serial.println("[BNO085] Failed I2C initialization");
        retVal = false;
    }

    if (!setReports())
    {
        Serial.println("[BNO085] Failed to set reports");
        retVal = false;
    }

    return retVal;
}

bool Bno085::setReports()
{
    bool retVal = true;

    if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR, UPDATE_TIME_US))
    {
        retVal = false;
    }

    return retVal;
}


bool Bno085::runLoop()
{
    if (bno085.wasReset())
    {
        Serial.println("[BNO085] ERROR: Sensor was reset");
        setReports();
        // Communicate reset to the pi via setting all to 0: Unlikely that all are exactly 0 and in the rare case they are, we can accept that the value will not be used (it should work fine)
        rotationVector.floats[Quaternion::term_real] = 0;
        rotationVector.floats[Quaternion::term_i] = 0;
        rotationVector.floats[Quaternion::term_j] = 0;
        rotationVector.floats[Quaternion::term_k] = 0;
        return true;
    }

    if (!bno085.getSensorEvent(&sensorValue))
    {
        // No new values yet
        return false;
    }

    switch (sensorValue.sensorId)
    {
        case SH2_GAME_ROTATION_VECTOR:
            rotationVector.floats[Quaternion::term_real] = sensorValue.un.gameRotationVector.real;
            rotationVector.floats[Quaternion::term_i] = sensorValue.un.gameRotationVector.i;
            rotationVector.floats[Quaternion::term_j] = sensorValue.un.gameRotationVector.j;
            rotationVector.floats[Quaternion::term_k] = sensorValue.un.gameRotationVector.k;
            return true;
            break;
        default:
            return false;
            break;
    }
}