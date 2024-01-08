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
        // Performs all required initialization
        void init();

        // Checks if new data is available (cheap, only checks local flag)
        bool check();

        // Compose the TransferData byteArray and update the registers with it
        void updateByteArray();

        // Set the settings to TransferData and decompose
        void updateSettings();

        // Getters for settings (call decomposeSettings first!)
        // void getRpmVals(int rpmVals[]);

        // For data sending
        TransferData transData {};

    private:
        // Constants
        // Number of motors
        static const int MOTOR_NUM = 4;
        // Length of often transmitted data (in bytes)
        static const int DATA_LEN = 64;
        // Length of data infrequently transmitted (in bytes)
        static const int INFREQ_DATA_LEN = 1;
        // Length of data frequently transmitted to control the teensy (in bytes)
        static const int CONTROL_DATA_LEN = 8;

        // Structs for doing i2c with registers
        // Controlling the teensy
        struct Settings
        {
            uint8_t writeRdyFlag = 1; // Whether the data can be written or not // REG0
            uint8_t filler1 = 0; // Reserved to fill memory up to 2-byte boundary // REG1
            uint8_t controlArr[CONTROL_DATA_LEN] {0}; // REG 2-9
        };

        // Sending to the pi
        struct Registers
        {
            uint8_t dataRdyFlag = 0; // Tells the master if data is ready or not // REG10
            uint8_t filler1 = 0; // Reserved to fill memory up to 2-byte boundary // REG11
            uint8_t byteArr[DATA_LEN] {}; // The data that should be retrieved often // REG12-75
            uint8_t infrequentArr[INFREQ_DATA_LEN] {}; // REG76-??
        };

        // Instantiate the structs.
        Settings settings {};
        Registers registers {};
        
        // 1 if new settings available, otherwise 0
        uint8_t newDataAvail = 0;

        // I2C stuff
        I2CRegisterSlave i2cSlave = I2CRegisterSlave(Slave1, (uint8_t*)&settings, sizeof(Settings), (uint8_t*)&registers, sizeof(Registers));
        void onReadISR(uint8_t regNum);
        void onWriteISR(uint8_t regNum, size_t numBytes);
        const uint8_t I2C_ADDRESS = 0x69;


};

#endif // COMMUNICATOR_H