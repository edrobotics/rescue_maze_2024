#pragma once

#include <math.h>
#include <iomanip>
#include <iostream>

#include "fusion/teensyCommunicator.h"

class Imu
{
    public:
        // Constructor
        // communicator: Pointer to teensycommunicator object responsible for communication
        Imu(TeensyCommunicator* communicator);

        // Gets new values from teensycommunicator and calculates euler angles
        void updateVals();

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

    private:
        float vals[TransferData::imu_num];
        Quaternion quatVals {};
        TeensyCommunicator* communicator;

        // Converts a quaternion into euler angles with YZX rotation order
        EulerAngle quaternionToEuler(Quaternion q);
        EulerAngle radToDeg(EulerAngle angle);

};