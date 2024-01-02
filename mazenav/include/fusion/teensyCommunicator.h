#pragma once
#include <iostream>
#include <vector>
#include <stdint.h>
#include <thread>
#include <chrono>

#include "fusion/i2cCommunicator.h"
#include "fusion/TransferData.h"

// For communicating with teensy. Backbone for many sensor-classes.
class TeensyCommunicator
{
    public:
        // portNum - the port of the i2c bus
        // addr - the slave address to communicate with
        TeensyCommunicator(uint8_t portNum, uint8_t addr);

        bool initiate();
        void testI2C();
        void test();

        TransferData transData {};

        // The registers on the teensy
        enum i2cRegisters
        {
            reg_rpmVals = 0,
            reg_rdyFlag = 8,
            reg_byteArr = 10,
            reg_infreqArr = 74,
            reg_num,
        };

        // Motor and wheel stuff
        // bool setWheelSpeed(float rpm);
        // int getWheelSpeed();
        // int getDistanceTravelled();

    private:
        i2cCommunicator i2cComm;
        const int tofReadingNum = 5; // Maximum number of TOF frames to accept.




};