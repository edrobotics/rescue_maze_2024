#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>
// #include "i2c_driver.h"
// #include "i2c_driver_wire.h"
#include "i2c_register_slave.h"

#include "TransferData.h"

class Communicator
{
    public:
        Communicator(int port);
        // Initialization
        void init();

        // Checks if new data is available (cheap, only checks local flag)
        bool check();

        void updateRegisters();

        // Settings control
        // Set the settings to TransferData and decompose
        void updateSettings();

        // Getters for settings (call decomposeSettings first!)
        void getRpmVals(int rpmVals[]);

        // For data sending
        TransferData transDat {};

    private:
        // I2C stuff
        static const int motorNum = 4;
        static const int dataLen = 64; // Number of bytes for data to be sent regurarly
        static const int infrequentDataLen = 1; // Number of bytes for data to be sent infrequently (battery voltage etc.)
        static const int CONTROL_DATA_LEN = 8;

        // Structs for doing i2c with registers
        // For controlling the teensy
        struct Settings
        {
            uint8_t writeRdyFlag = 1; // Whether the data can be written or not // REG0
            uint8_t filler1 = 0; // Reserved to fill memory up to 2-byte boundary // REG1
            uint8_t controlArr[CONTROL_DATA_LEN] {0}; // REG 2-9
        };

        struct Registers
        {
            uint8_t dataRdyFlag = 0; // Tells the master if data is ready or not // REG10
            uint8_t filler1 = 0; // Reserved to fill memory up to 2-byte boundary // REG11
            uint8_t byteArr[dataLen] {}; // The data that should be retrieved often // REG12-75
            uint8_t infrequentArr[infrequentDataLen] {}; // REG76-??
        };

        // Instantiate the structs.
        Settings settings {};
        Registers registers {};

        I2CRegisterSlave i2cSlave = I2CRegisterSlave(Slave1, (uint8_t*)&settings, sizeof(Settings), (uint8_t*)&registers, sizeof(Registers));
        void onReadISR(uint8_t regNum);
        void onWriteISR(uint8_t regNum, size_t numBytes);
        const uint8_t I2C_ADDRESS = 0x69;
        uint8_t newDataAvail = 0;


};

#endif // COMMUNICATOR_H