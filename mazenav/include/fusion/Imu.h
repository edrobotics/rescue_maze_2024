#pragma once

#include <math.h>
#include <iomanip>
#include <iostream>
#include "fusion/mutexes.h"
#include <mutex>

#include "fusion/TeensyCommunicator.h"

class Imu
{
    public:
        // Constructor
        // communicator: Pointer to teensycommunicator object responsible for communication
        Imu(TeensyCommunicator* communicator);

        // Gets new values from teensycommunicator and calculates euler angles
        // Returns true if the values have been updated
        bool updateVals();

        // Print the values
        // newLine - whether or not to print the newline
        void printVals(bool newLine);

        struct Quaternion
        {
            double real {0};
            double i {0};
            double j {0};
            double k {0};
        };

        struct EulerAngle
        {
            double x {0};
            double y {0};
            double z {0};
        };

        // The data to use outward. Euler angles with YZX rotation order, left hand coordinate system
        EulerAngle angles {};

        // Returns true if an IMU reset was detected, otherwise false
        bool getWasReset();

    private:

        // For TS access control
        std::mutex mtx_general {};

        float vals[TransferData::imu_num];
        Quaternion quatVals {};
        TeensyCommunicator* communicator;

        // Converts a quaternion into euler angles with sensor local YZX rotation order, global convention XZY rotation order
        EulerAngle quaternionToEuler(Quaternion q);
        EulerAngle radToDeg(EulerAngle angle);

        bool wasReset {false};

};