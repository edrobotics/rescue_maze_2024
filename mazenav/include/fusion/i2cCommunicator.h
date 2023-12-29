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
    i2cCommunicator(__uint8_t portNum, __uint8_t addr);
    bool init();
    bool readRegister(__uint8_t reg, __uint8_t* value);
    bool writeRegister(__uint8_t reg, __uint8_t value);

    private:
    __uint8_t portNum;
    __uint8_t slaveAddr;
    char filename[20];
    int i2cFile;

    bool readReg(int file, __uint8_t addr, __uint8_t reg, __uint8_t* value);
    bool writeReg(int file, __uint8_t addr, __uint8_t reg, __uint8_t value);

};