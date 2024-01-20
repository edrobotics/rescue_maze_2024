#pragma once

#include <iostream>
#include <iomanip>
#include <mutex>
#include "fusion/mutexes.h"

#include "fusion/teensyCommunicator.h"

class Tof
{
    public:
        Tof(TeensyCommunicator* communicator);

        // Get new values from teensycommunicator
        void updateVals();

        // Prints the values
        void printVals(bool newline);

        struct TofData
        {
            int lf {0}; // Left front
            int lb {0}; // Left back
            int rf {0}; // Right front
            int rb {0}; // Right back

            int fl {0}; // Front left
            int fr {0}; // Front right
            int b {0}; // Back
        };

        enum TofID
        {
            tof_lf,
            tof_lb,
            tof_rf,
            tof_rb,
            tof_fl,
            tof_fr,
            tof_b,
            tof_num,
        };

        TofData tofData {};

    private:
        TeensyCommunicator* communicator;
        uint16_t vals[tof_num] {0};;

};