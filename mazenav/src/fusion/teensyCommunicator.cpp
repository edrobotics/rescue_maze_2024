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


void TeensyCommunicator::runLoop()
{
    writeSettings();

    readFrequent();
}

void TeensyCommunicator::runLoopLooper()
{
    while(true)
    {
        runLoop();
    }
}

bool TeensyCommunicator::writeSettings()
{
    mtx_transData_controlData.lock();
    transData.composeSettings();
    mtx_transData_controlData.unlock(); // Operating on byte array after this
    if (!i2cComm.writeRegister(reg_controlVals, transData.SETTING_LEN, transData.controlArr))
    {
        return false;
    }

    uint8_t dataWritten[] {1};
    if (!i2cComm.writeRegister(reg_dataWritten, 1, dataWritten))
    {
        return false;
    }

    return true;

}

bool TeensyCommunicator::readFrequent()
{
    // Check data ready. Max loop limit implemented.
    bool cont = false; // Whether to continue or not
    while (cont==false)
    {
        // static int iterations {0};
        cont = checkRdy();
        // ++iterations;
        // if (iterations>20)
        // {
        //     cont = true;
        // }
    }

    if (!i2cComm.readRegister(reg_byteArr, transData.DATA_LEN, transData.byteArr))
    {
        return false;
    }
    uint8_t dataRead[] {1};
    if (!i2cComm.writeRegister(reg_dataRead, 1, dataRead))
    {
        return false;
    }

    mtx_transData_freqData.lock();
    transData.decompose();
    mtx_transData_freqData.unlock();
    return true;
}

bool TeensyCommunicator::checkRdy()
{
    uint8_t result[] {0};
    i2cComm.readRegister(reg_rdyFlag, 1, result);
    if (result[0]==1)
    {
        return true;
    }
    else
    {
        return false;
    }
}


void TeensyCommunicator::testI2C()
{
}

void TeensyCommunicator::test()
{
    // transData.test();
    // uint8_t byteArray[transData.DATA_LEN] {};
    // uint8_t dataRdy[] {0};
    // while (dataRdy[0] != 1)
    // {
    //     i2cComm.readRegister(reg_rdyFlag, 1, dataRdy);
    // }
    // i2cComm.readRegister(reg_byteArr, transData.DATA_LEN, byteArray);
    // memcpy(transData.byteArr, byteArray, transData.DATA_LEN);
    // transData.decompose();
    // float real = 0;
    // float i = 0;
    // float j = 0;
    // float k = 0;
    // transData.getIMU(0, TransferData::imu_real, real);
    // transData.getIMU(0, TransferData::imu_i, i);
    // transData.getIMU(0, TransferData::imu_j, j);
    // transData.getIMU(0, TransferData::imu_k, k);
    
    // std::cout << "real=" << real << "  i=" << i << "  j=" << j << "  k=" << k << "\n";
    // int motorSpeeds = 0;
    // for (int i=0;i<4;++i)
    // {
    //     transData.setRpmControl(i, motorSpeeds);
    // }
    // transData.composeSettings();

    // uint8_t settingArr[8] {0};
    // transData.getControlArr(settingArr);
    // std::cout << "SettingArr: ";
    // for (int i=0;i<8;++i)
    // {
    //     std::cout << settingArr[i] << ",";
    // }
    // std::cout << "\n";

    // i2cComm.writeRegister(reg_rpmVals, sizeof(settingArr), settingArr);

    // std::cout << "Registers written\n";



    // char userInput {'n'};
    // std::cout << "Command: ";
    // std::cin >> userInput;
    // switch (userInput)
    // {
    //     case 'w':
    //         motorSpeeds = 70;
    //         break;
    //     case 's':
    //         motorSpeeds = -70;
    //         break;
    //     case ' ':
    //     default:
    //         motorSpeeds = 0;
    //         break;
    // }

    // for (int i=0;i<4;++i)
    // {
    //     transData.setRpmControl(i, motorSpeeds);
    // }
    // transData.composeSettings();

    // // uint8_t settingArr[8] {0};
    // transData.getControlArr(settingArr);
    // std::cout << "SettingArr: ";
    // for (int i=0;i<8;++i)
    // {
    //     std::cout << settingArr[i] << ",";
    // }
    // std::cout << "\n";

    // i2cComm.writeRegister(reg_rpmVals, sizeof(settingArr), settingArr);

    // std::cout << "Registers written\n";

    // if (motorSpeeds != 0)
    // {
    //     std::chrono::time_point timeFlag = std::chrono::steady_clock::now();
    //     while(std::chrono::steady_clock::now()-timeFlag < std::chrono::milliseconds(600))
    //     {
    //         // Wait until data is ready
    //         std::cout << "Waiting for data...    ";
    //         static uint8_t rdyFlag[1] {0};
    //         while (rdyFlag[0] != 1)
    //         {
    //             i2cComm.readRegister(reg_rdyFlag, 1, rdyFlag);
    //             std::cout << "rdyFlag[0]=" << rdyFlag[0] << "\n";
    //             std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //         }
    //         std::cout << "Data Ready" << "\n";

    //         uint8_t byteArr[64] {};

    //         i2cComm.readRegister(reg_byteArr, 64, byteArr);
    //         memcpy(transData.byteArr, byteArr, 64);

    //         transData.decompose();

    //         std::cout << "Motor speeds are: ";
    //         for (int i=0;i<4;++i)
    //         {
    //             int motorSpeed {133};
    //             transData.getRPM(i, motorSpeed);
    //             std::cout << "M" << i << "=" << motorSpeed << "    ";
    //         }
    //         std::cout << "\n";
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //     }
    // }
}

