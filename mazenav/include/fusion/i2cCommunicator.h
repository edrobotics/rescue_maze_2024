#pragma once

#include <iostream>

// I2C includes
#include <fcntl.h> // File operations (I2C)
#include <linux/i2c-dev.h> // I2C
#include <i2c/smbus.h> // I2C
#include <sys/ioctl.h> // I2C
#include <stdint.h>

class i2cCommunicator
{
    public:
    i2cCommunicator(uint8_t portNum, uint8_t addr);
    bool init();
    bool readRegister(uint8_t reg, uint8_t size, uint64_t* value);
    bool writeRegister(uint8_t reg, uint8_t size, uint64_t value);
    bool readTransferData();

    private:
    uint8_t portNum;
    uint8_t slaveAddr;
    char filename[20];
    int i2cFile;

    bool readReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint64_t* value);
    bool writeReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint64_t value);

};