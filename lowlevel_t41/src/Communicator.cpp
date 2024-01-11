#include "Communicator.h"

Communicator::Communicator(int port)
{

}

void Communicator::onReadISR(uint8_t regNum)
{
    // Reset the ready flag to tell the master that the newest data has been read.
    // registers.dataRdyFlag = 0;
}

void Communicator::onWriteISR(uint8_t regNum, size_t numBytes)
{
    // If the pi has said that it has read all data, reset the flag so that it can be set next time
    if (settings.dataRead==1)
    {
        registers.dataRdyFlag = 0;
    }
}

void Communicator::init()
{
    // Begin listening on I2C_ADDRESS
    i2cSlave.listen(I2C_ADDRESS);

    // Register I2C isr functions
    i2cSlave.after_read(std::bind(&Communicator::onReadISR, this, std::placeholders::_1));
    i2cSlave.after_write(std::bind(&Communicator::onWriteISR, this, std::placeholders::_1, std::placeholders::_2));

    Serial.println("[Communicator] I2C initialized");
}

bool Communicator::checkWrite()
{
    if (settings.dataWritten==1)
    {
        settings.dataWritten = 0;
        return true;
    }
    else
    {
        return false;
    }
}

void Communicator::updateSettings()
{
    memcpy(transData.controlArr, settings.controlArr, CONTROL_DATA_LEN);
    transData.decomposeSettings();
}

void Communicator::updateByteArray()
{
    registers.dataRdyFlag = 0;
    uint8_t byteArr[DATA_LEN] {};

    transData.compose();
    transData.getByteArr(byteArr);

    memcpy(registers.byteArr, byteArr, DATA_LEN);
    registers.dataRdyFlag = 1;
}