#pragma once
#include "TofVl53l0x.h"
#include "TofVl53l1x.h"
#include <DFRobot_MCP23017.h>
#include <Wire.h>


// Usage description:
// First initialize the collection. This will perform necessary initialization and begin continuous measurement (periodic).
// When you want data, call update() to pull new data from the sensors. Once update() returns true, read the distance array (distances[]).
// When update returns true, internal flags for how recent data is is reset, so it will not return true until all sensors have gathered new values.

class TofCollection
{
    public:
        // Constructor
        TofCollection(DFRobot_MCP23017* ioExpander);

        static const int L0X_NUM {4};
        static const int L1X_NUM {3};
        static const int TOF_NUM {L0X_NUM+L1X_NUM};

        // Initializes all sensors
        // Returns:
        // - Number representing how many of the sensors failed
        int init();

        // Tries to read new values from the sensors.
        // Return:
        // true - all sensors were read and updated
        // false - one or more sensors were not updated. Call again until returning true (will check just the unsuccessful sensors)
        bool update();

        // Distance array that can be read once update has returned true (otherwise risk reading old values)
        uint8_t distances[TOF_NUM] {0};

        void test();


        // Should not be public, but did not work as private
        DFRobot_MCP23017::ePin_t pins[TOF_NUM] {DFRobot_MCP23017::eGPA7, DFRobot_MCP23017::eGPB0, DFRobot_MCP23017::eGPB6, DFRobot_MCP23017::eGPB2, DFRobot_MCP23017::eGPB1, DFRobot_MCP23017::eGPB4, DFRobot_MCP23017::eGPB3};
    private:
        DFRobot_MCP23017* ioExpander;
        TofVl53l0x l0xArr[L0X_NUM] {};
        TofVl53l1x l1xArr[L1X_NUM] {};
        const int L0X_SAMPLING_TIME {20000};
        const int L1X_SAMPLING_TIME {20000};

        struct TofData
        {
            uint16_t distance {0};
            bool dataRdy {false};
        };

        TofData tofData[TOF_NUM] {};

        // Array to store all sensor addresses
        const int START_ADDR {0x29};
        uint8_t addresses[TOF_NUM] {};
        // Compute the addresses
        // Starts at 0x29 (default address) and increments by one for each sensor
        void fillAddr();

        // Resets the sensors
        void resetSensors();

        // Disables all sensors
        void disableSensors();


};

