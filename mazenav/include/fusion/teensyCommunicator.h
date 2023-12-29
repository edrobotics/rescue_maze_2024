#pragma once
#include <iostream>
#include <vector>
#include <stdint.h>
#include <thread>
#include <chrono>

#include "fusion/i2cCommunicator.h"

// For communicating with teensy. Backbone for many sensor-classes.
class TeensyCommunicator
{
    public:
        // portNum - the port of the i2c bus
        // addr - the slave address to communicate with
        TeensyCommunicator(uint8_t portNum, uint8_t addr);

        bool initiate();
        void testI2C();

        struct Settings
        {
            uint8_t operation; // 0 is addition, 1 is subtraction
            uint16_t number1; // The first number to execute the operation on
            uint16_t number2; // The other number to execute the operation on
        };

        struct Registers
        {
            uint8_t readyFlag = 0; // Tells the master if data is ready or not
            int16_t result = 420;
        };

        // // Motor and wheel stuff
        // bool setWheelSpeed(float rpm);
        // int getWheelSpeed();
        // int getDistanceTravelled();

    private:
        i2cCommunicator i2cComm;
        const int tofReadingNum = 5; // Maximum number of TOF frames to accept.



};