#include "Communicator.h"

Communicator::Communicator(int port)
{

}

void Communicator::onReadISR(uint8_t regNum)
{
    // Reset the ready flag to tell the master that the newest data has been read.
    registers.dataRdyFlag = 0;
}

void Communicator::onWriteISR(uint8_t regNum, size_t numBytes)
{
    newDataAvail = 1;
}

void Communicator::init()
{
    // Begin listening on I2C_ADDRESS
    i2cSlave.listen(I2C_ADDRESS);

    // Register a function to run after reading
    i2cSlave.after_read(std::bind(&Communicator::onReadISR, this, std::placeholders::_1));

    i2cSlave.after_write(std::bind(&Communicator::onWriteISR, this, std::placeholders::_1, std::placeholders::_2));
    Serial.println("I2C initialized");
}

bool Communicator::check()
{
    if (newDataAvail==1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Communicator::updateRegisters()
{
    Registers newRegisters {};
    newRegisters.dataRdyFlag = 1;

    transDat.compose();
    transDat.getByteArr(newRegisters.byteArr);
    newRegisters.infrequentArr[0] = 0;

    memcpy(&registers, &newRegisters, sizeof(Registers));

}

void Communicator::getRpmVals(int16_t rpmVals[motorNum])
{
    memcpy(settings.rpmVal, rpmVals, motorNum);
}