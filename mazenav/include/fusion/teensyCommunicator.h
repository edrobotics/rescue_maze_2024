#pragma once
#include <iostream>
#include <vector>
#include <stdint.h>
#include <thread>
#include <chrono>

#include "fusion/i2cCommunicator.h"
#include "TransferData/TransferData.h"

// For communicating with teensy. Backbone for many sensor-classes.
class TeensyCommunicator
{
    public:
        // portNum - the port of the i2c bus
        // addr - the slave address to communicate with
        TeensyCommunicator(uint8_t portNum, uint8_t addr);

        bool initiate();

        // Run this loop to update data once. It compiles and decompiles the data when appropriate.
        void runLoop();
        
        // Testing
        void testI2C();
        void test();

        TransferData transData {};

        // The registers on the teensy

        // Motor and wheel stuff
        // bool setWheelSpeed(float rpm);
        // int getWheelSpeed();
        // int getDistanceTravelled();

    private:
        i2cCommunicator i2cComm;
        enum i2cRegisters
        {
            reg_dataWritten = 0,
            reg_dataRead = 1,
            reg_controlVals = 2,
            reg_rdyFlag = 10,
            reg_byteArr = 12,
            reg_infreqArr = 76,
            reg_num,
        };
        bool writeSettings();
        bool readFrequent();
        bool readInfrequent();
        bool checkRdy();


};