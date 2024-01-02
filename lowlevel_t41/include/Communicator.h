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

        // Getters for settings
        void getRpmVals(int16_t rpmVals[]);

        // For data sending
        TransferData transDat {};

    private:
        // I2C stuff
        static const int motorNum = 4;
        static const int dataLen = 64; // Number of bytes for data to be sent regurarly
        static const int infrequentDataLen = 1; // Number of bytes for data to be sent infrequently (battery voltage etc.)

        // Structs for doing i2c with registers
        // For controlling the teensy
        struct Settings
        {
            int16_t rpmVal[motorNum] {}; // REG 0-1, 2-3, 4-5, 6-7
        };

        struct Registers
        {
            uint8_t dataRdyFlag = 0; // Tells the master if data is ready or not // REG8
            uint8_t filler1 = 0; // Reserved to fill memory up to 2-byte boundary // REG9
            uint8_t byteArr[dataLen] {}; // The data that should be retrieved often // REG10-73
            uint8_t infrequentArr[infrequentDataLen] {}; // REG74-??
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