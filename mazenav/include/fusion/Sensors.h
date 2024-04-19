#pragma once

#include "fusion/TeensyCommunicator.h"
#include "fusion/Imu.h"
#include "fusion/Tof.h"
#include "fusion/MotorControllers.h"
#include "fusion/ColourSensor.h"

class Sensors
{
    public:
        Sensors(TeensyCommunicator* communicator, i2cCommunicator* i2cComm);

        void init();

        // Update all sensors.
        // capture - if true, wait for all to be updated. If false, return regardless of update state.
        void update(bool capture);

        // Print out the sensor values in a nice format
        void print();

        // All initialized in constructor
        Imu imu0;
        Tof tofs;
        MotorControllers motors;
        ColourSensor colSens;


    private:
        TeensyCommunicator* communicator;


};