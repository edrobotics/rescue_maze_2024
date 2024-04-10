#pragma once

#include <iostream>
#include <iomanip>
#include <mutex>
#include "fusion/mutexes.h"

#include "fusion/TeensyCommunicator.h"

class Tof
{
    public:
        Tof(TeensyCommunicator* communicator);

        // Get new values from teensycommunicator.
        // Returns true if the values have been updated
        bool updateVals();

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
            tof_lf = 2,
            tof_lb = 1,
            tof_rf = 3,
            tof_rb = 0,
            tof_fl = 5,
            tof_fr = 6,
            tof_b  = 4,
            tof_num = 7,
        };

        TofData tofData {};

    private:
        // For TS access control
        std::mutex mtx_general {};
        TeensyCommunicator* communicator;
        uint16_t vals[tof_num] {0};

};