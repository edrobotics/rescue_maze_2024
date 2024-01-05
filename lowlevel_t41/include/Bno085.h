#pragma once
#include <Adafruit_BNO08x.h>
#include "Quaternion.h"

// Wrapper for the adafruit bno08x class
class Bno085
{
    public:
        Bno085(int address);
        Adafruit_BNO08x bno085 {};
        Quaternion rotationVector {};

        // All required initialization
        // Return:
        //  true - success
        //  false - fail
        bool init();

        // Loop to update sensor and values.
        // Return:
        //  true - values were updated
        //  false - not enough time passed or some error occured
        bool runLoop();

    private:
        int i2cAddr = -1;
        // To store incoming sensor data
        sh2_SensorValue_t sensorValue {};
        
        // Set the wanted reports in the sensor
        bool setReports();

        // Time interval between sensor updates
        const int UPDATE_TIME_US = 10000;

};