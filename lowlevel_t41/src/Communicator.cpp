#include "Communicator.h"

Communicator::Communicator(int port)
{

}

void Communicator::onReadISR(uint8_t regNum)
{
    // Reset the ready flag to tell the master that the newest data has been read.
    registers.readyFlag = 0;
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

void Communicator::recompute()
{
    Registers newData;
    newData.readyFlag = 1;
    newDataAvail = 0;
    if (settings.operation == 0)
    {
        newData.result = settings.number1 + settings.number2*1030;
    }
    else if (settings.operation == 1)
    {
        newData.result = settings.number1 - settings.number2;
    }
    else
    {
        newData.result = 0;
    }
    Serial.println("Recomputing:");
    Serial.print("Num1=");Serial.print(settings.number1);Serial.print(" Num2=");Serial.print(settings.number2);Serial.print(" Operation=");Serial.println(settings.operation);
    Serial.print("Result is: ");Serial.println(static_cast<int>(newData.result));

    memcpy(&registers, &newData, sizeof(Registers));
    Serial.print("registers.result=");Serial.println(registers.result);
}