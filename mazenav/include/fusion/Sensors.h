#pragma once

#include "fusion/TeensyCommunicator.h"
#include "fusion/Imu.h"
#include "fusion/Tof.h"
#include "fusion/MotorControllers.h"

class Sensors
{
    public:
        Sensors(TeensyCommunicator* communicator);

        void update();

        // Print out the sensor values in a nice format
        void print();

        // All initialized in constructor
        Imu imu0;
        Tof tofs;
        MotorControllers motors;


    private:
        TeensyCommunicator* communicator;


};