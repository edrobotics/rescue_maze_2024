#include "fusion/i2cCommunicator.h"

i2cCommunicator::i2cCommunicator(uint8_t portNum, uint8_t addr)
{
    this->portNum = portNum;
    this->slaveAddr = addr;
    snprintf(filename, 19, "/dev/i2c-%d", portNum);
    std::cout << "Constructed filename: " << filename << "\n";
}

bool i2cCommunicator::init()
{
    if ((i2cFile = open(filename, O_RDWR)) < 0)
    {
        return false;
    }
    if (ioctl(i2cFile, I2C_SLAVE, slaveAddr) < 0)
    {
        return false;
    }

    return true;


}

bool i2cCommunicator::readRegister(uint8_t reg, uint8_t* value)
{
    return readReg(i2cFile, slaveAddr, reg, value);
}

bool i2cCommunicator::writeRegister(uint8_t reg, uint8_t value)
{
    return writeReg(i2cFile, slaveAddr, reg, value);
}

bool i2cCommunicator::readReg(int file, uint8_t addr, uint8_t reg, uint8_t* value)
{
    uint8_t inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf = reg;
    messages[0].addr = addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(outbuf);
    messages[0].buf = &outbuf;

    messages[1].addr = addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = sizeof(inbuf);
    messages[1].buf = &inbuf;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(file, I2C_RDWR, &packets) < 0)
    {
        return false;
    }

    *value = inbuf;

    return true;
}

bool i2cCommunicator::writeReg(int file, uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    outbuf[0] = reg;
    outbuf[1] = value;

    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        return 0;
    }

    return 1;
    return true;
}
