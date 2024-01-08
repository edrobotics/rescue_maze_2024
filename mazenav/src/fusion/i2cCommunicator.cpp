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

bool i2cCommunicator::readRegister(uint8_t reg, uint8_t size, uint8_t values[])
{
    return readReg(i2cFile, slaveAddr, reg, size, values);
}

bool i2cCommunicator::writeRegister(uint8_t reg, uint8_t size, uint8_t values[])
{
    return writeReg(i2cFile, slaveAddr, reg, size, values);
}

bool i2cCommunicator::readReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[])
{
    uint8_t outbuf;
    uint8_t inbuf[size];
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
    messages[1].buf = inbuf; // Pointer to the array?

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(file, I2C_RDWR, &packets) < 0)
    {
        return false;
    }

    // Return the byte array
    memcpy(values, inbuf, size);

    return true;
}

bool i2cCommunicator::writeReg(int file, uint8_t addr, uint8_t reg, uint8_t size, uint8_t values[])
{
    uint8_t outbuf[1+size];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    outbuf[0] = reg;
    for (int i=0;i<size;++i)
    {
        outbuf[i+1] = values[i];
    }

    packets.msgs  = messages;
    packets.nmsgs = 1;
    std::cout << "Before write\n";
    if (ioctl(file, I2C_RDWR, &packets) < 0)
    {
        std::cout << "Aborted\n";
        std::cout << "Errno: " << errno << "  with explanation " << strerror(errno) << "\n";
        return false;
    }
    std::cout << "After successful write\n";

    return true;
}
