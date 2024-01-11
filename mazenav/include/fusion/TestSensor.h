#pragma once
#include "fusion/teensyCommunicator.h"
#include <iostream>

class TestSensor
{
    public:
        TestSensor(TeensyCommunicator* communicator);
        void updateVals();
        void printVals();

    private:
        float imuVals[TransferData::imu_num] {0};
        TeensyCommunicator* communicator;
};