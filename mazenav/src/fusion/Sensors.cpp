#include "fusion/Sensors.h"

Sensors::Sensors(TeensyCommunicator* communicator)
    : imu0 {communicator}, tofs {communicator}, motors {communicator}
{
    this->communicator = communicator;
}

void Sensors::update(bool capture)
{
    if (capture)
    {
        bool imuUpdated {false};
        bool tofUpdated {false};
        bool motorsUpdated {false};

        motors.setVals();

        // While at least one not updated exists
        while (!imuUpdated || !tofUpdated || !motorsUpdated)
        {
            if (!imuUpdated)
            {
                imuUpdated = imu0.updateVals();
            }

            if (!tofUpdated)
            {
                tofUpdated = tofs.updateVals();
            }

            if (!motorsUpdated)
            {
                motorsUpdated = motors.getVals();
            }
        }
        // tofs.printVals(true);

    }
    else
    {
        imu0.updateVals();
        tofs.updateVals();
        motors.getVals();
        motors.setVals();
    }
}

void Sensors::print()
{
    imu0.printVals(false);
    std::cout << "    ";
    tofs.printVals(false);
    std::cout << "\n";
}