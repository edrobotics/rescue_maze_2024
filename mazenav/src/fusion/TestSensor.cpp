#include "fusion/TestSensor.h"

TestSensor::TestSensor(TeensyCommunicator* communicator)
{
    this->communicator = communicator;
}

void TestSensor::updateVals()
{
    communicator->transData.getIMU(0, imuVals);
}

void TestSensor::printVals()
{
    std::cout << "real=" << imuVals[0] << "  ";
    std::cout << "i=" << imuVals[1] << "  ";
    std::cout << "j=" << imuVals[2] << "  ";
    std::cout << "k=" << imuVals[3] << "  ";
    std::cout << "\n";
}