#pragma once

#include "fusion/teensyCommunicator.h"
#include "fusion/Imu.h"
#include "fusion/Tof.h"
#include "fusion/MotorControllers.h"

class Sensors
{
    public:
        Sensors(TeensyCommunicator* communicator);

        void update();

        Imu imu0;
        Tof tofs;


    private:
        TeensyCommunicator* communicator;


};