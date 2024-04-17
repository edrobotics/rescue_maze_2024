#pragma once

#include <iostream>

// I2C includes
#include <fcntl.h> // File operations (I2C)
#include <linux/i2c-dev.h> // I2C
#include <i2c/smbus.h> // I2C
#include <sys/ioctl.h> // I2C
#include <stdint.h> // Types
#include <cstring> // memcpy

class i2cCommunicator
{
    public:
    i2cCommunicator(uint8_t portNum);
    bool init();
    // bool readRegister(uint8_t reg, uint8_t size, uint8_t values[]);
    // bool writeRegister(uint8_t reg, uint8_t size, uint8_t values[]);

    bool readReg(uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[]);
    bool writeReg(uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[]);

    private:
    uint8_t portNum;
    // uint8_t slaveAddr;
    char filename[20];
    int i2cFile;

    bool readReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[]);
    bool writeReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[]);

};