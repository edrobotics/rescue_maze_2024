#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>
// #include "i2c_driver.h"
// #include "i2c_driver_wire.h"
#include "i2c_register_slave.h"

class Communicator
{
    public:
    Communicator(int port);
    void init();
    void recompute();
    bool check();

    // Structs for doing i2c with registers (may be used later?)
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

    // Instantiate the structs.
    // IDEA: Have two structs (=two sets of values). Then have a pointer to one struct. Change only the pointer to change which struct is accessed? Faster updating than memcpy.(?)
    Settings settings {0, 0};
    Registers registers;
    


    private:
    I2CRegisterSlave i2cSlave = I2CRegisterSlave(Slave1, (uint8_t*)&settings, sizeof(Settings), (uint8_t*)&registers, sizeof(Registers));
    void onReadISR(uint8_t regNum);
    void onWriteISR(uint8_t regNum, size_t numBytes);
    const uint8_t I2C_ADDRESS = 0x69;
    uint8_t newDataAvail = 0;

};

#endif // COMMUNICATOR_H