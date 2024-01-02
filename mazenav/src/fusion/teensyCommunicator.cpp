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
}

void TeensyCommunicator::test()
{
    for (int i=0;i<4;++i)
    {
        transData.setRpmControl(i, 50);
    }
    transData.composeSettings();

    uint8_t settingArr[8] {};
    transData.getSettingsArr(settingArr);

    i2cComm.writeRegister(reg_rpmVals, sizeof(settingArr), settingArr);

    while (true)
    {
        // Wait until data is ready
        std::cout << "Waiting for data...    ";
        static uint8_t rdyFlag[1] {0};
        while (rdyFlag[0] != 1)
        {
            i2cComm.readRegister(reg_rdyFlag, 1, rdyFlag);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "Data Ready" << "\n";

        uint8_t byteArr[64] {};

        i2cComm.readRegister(reg_byteArr, 64, byteArr);

        memcpy(transData.byteArr, byteArr, 64);

        transData.decompose();

        std::cout << "Motor speeds are: ";
        for (int i=0;i<4;++i)
        {
            int motorSpeed {133};
            transData.getRPM(i, motorSpeed);
            std::cout << "M" << i << "=" << motorSpeed << "    ";
        }
        std::cout << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

