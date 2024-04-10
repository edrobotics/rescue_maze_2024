#pragma once

#include <iostream>
#include <iomanip>
#include <mutex>
#include "fusion/mutexes.h"

#include "fusion/TeensyCommunicator.h"
#include "fusion/SingleTofData.h"

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
            SingleTofData lf {}; // Left front
            SingleTofData lb {}; // Left back
            SingleTofData rf {}; // Right front
            SingleTofData rb {}; // Right back

            SingleTofData fl {}; // Front left
            SingleTofData fr {}; // Front right
            SingleTofData b {}; // Back
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