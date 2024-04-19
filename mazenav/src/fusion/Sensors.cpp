#include "fusion/Sensors.h"

Sensors::Sensors(TeensyCommunicator* communicator, i2cCommunicator* i2cComm)
    : imu0 {communicator}, tofs {communicator}, motors {communicator}, colSens {i2cComm}
{
    this->communicator = communicator;
}

void Sensors::init()
{
    colSens.init();
}

void Sensors::update(bool capture)
{
    if (capture)
    {
        bool imuUpdated {false};
        bool tofUpdated {false};
        bool motorsUpdated {false};
        bool colourUpdated {false};

        motors.setVals();

        // While at least one not updated exists
        while (!imuUpdated || !tofUpdated || !motorsUpdated || !colourUpdated)
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

            if (!colourUpdated)
            {
                colourUpdated = colSens.updateVals();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
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