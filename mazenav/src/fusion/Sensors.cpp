#include "fusion/Sensors.h"

Sensors::Sensors(TeensyCommunicator* communicator)
    : imu0 {communicator}, tofs {communicator}
{
    this->communicator = communicator;
}

void Sensors::update()
{
    imu0.updateVals();
    tofs.updateVals();
}