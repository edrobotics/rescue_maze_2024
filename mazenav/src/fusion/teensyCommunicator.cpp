#include <fusion/teensyCommunicator.h>

TeensyCommunicator::TeensyCommunicator(uint8_t portNum, uint8_t addr)
    : i2cComm{portNum, addr}
{

}

bool TeensyCommunicator::initiate()
{
    if(!i2cComm.init())
    {
        std::cout << "Initialization of I2C failed";
        return false;
    }

    return true;
}

void TeensyCommunicator::testI2C()
{
    i2cComm.writeRegister(0x00, 1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    i2cComm.writeRegister(0x01, 1, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    i2cComm.writeRegister(0x02, 1, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    uint64_t dataRdy = 0;
    while (dataRdy!=1)
    {
        i2cComm.readRegister(0x03, 1, &dataRdy);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    uint64_t result = -69;
    i2cComm.readRegister(0x05, 2, &result);
    std::cout << "The result is: " << result << "\n";
}

