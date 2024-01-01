#include "fusion/TransferData.h"


TransferData::TransferData()
{

}

void TransferData::compose()
{
    int arrIdx = 0;
    // ToF
    for (int i=0;i<tofNum;++i)
    {
        u16toB(tofData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

    // Colour sensors
    for (int i=0;i<colNum;++i)
    {
        for (int k=0;k<col_num;++k)
        {
            u16toB(colData[i][k], byteArr[arrIdx], byteArr[arrIdx+1]);
            arrIdx+=2;
        }
    }

    // IMU
    for (int i=0;i<imuNum;++i)
    {
        for (int k=0;k<imu_num;++k)
        {
            s16toB(imuData[i][k], byteArr[arrIdx], byteArr[arrIdx+1]);
            arrIdx+=2;
        }

    }

    //RPM
    for (int i=0;i<motorNum;++i)
    {
        s16toB(rpmData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

    //Pos
    for (int i=0;i<motorNum;++i)
    {
        s16toB(posData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

}

void TransferData::decompose()
{
    int arrIdx = 0;
    // ToF
    for (int i=0;i<tofNum;++i)
    {
        btoU16(byteArr[arrIdx], byteArr[arrIdx+1], tofData[i]);
        arrIdx+=2;
    }

    // Colour sensors
    for (int i=0;i<colNum;++i)
    {
        for (int k=0;k<col_num;++k)
        {
            btoU16(byteArr[arrIdx], byteArr[arrIdx+1], colData[i][k]);
            arrIdx+=2;
        }
    }

    // IMU
    for (int i=0;i<imuNum;++i)
    {
        for (int k=0;k<imu_num;++k)
        {
            btoS16(byteArr[arrIdx], byteArr[arrIdx+1], imuData[i][k]);
            arrIdx+=2;
        }

    }

    //RPM
    for (int i=0;i<motorNum;++i)
    {
        btoS16(byteArr[arrIdx], byteArr[arrIdx+1], rpmData[i]);
        arrIdx+=2;
    }

    //Pos
    for (int i=0;i<motorNum;++i)
    {
        btoS16(byteArr[arrIdx], byteArr[arrIdx+1], posData[i]);
        arrIdx+=2;
    }

}


void TransferData::u16toB(uint16_t input, uint8_t& byte1, uint8_t& byte2)
{
    byte1 = static_cast<uint8_t>((input & (0b11111111 << 8)) >> 8);
    byte2 = static_cast<uint8_t>(input & 0b11111111);
}

void TransferData::s16toB(int16_t input, uint8_t& byte1, uint8_t& byte2)
{
    byte1 = static_cast<uint8_t>((input & (0b11111111 << 8)) >> 8);
    byte2 = static_cast<uint8_t>(input & 0b11111111);
}

void TransferData::btoU16(uint8_t input1, uint8_t input2, uint16_t& output)
{
    output = static_cast<uint16_t>(input2);
    output |= input1 << 8;
}

void TransferData::btoS16(uint8_t input1, uint8_t input2, int16_t& output)
{
    output = static_cast<uint16_t>(input2);
    output |= input1 << 8;
}

void TransferData::test()
{
    // uint16_t u16 = 555;
    // int16_t s16 = -555;
    // uint8_t byte1, byte2, byte3, byte4;
    // s16toB(s16, byte1, byte2);
    // std::cout << "s16=" << s16 << "\n";
    // btoS16(byte1, byte2, s16);
    // std::cout << "s16=" << s16 << "\n";

    

}



bool TransferData::getTof(int index, int& value)
{
    if (index<0 || index >= tofNum)
    {
        return false;
    }
    else
    {
        value = tofData[index];
        return true;
    }

}

bool TransferData::getCol(int index, Col colour, int& value)
{
    if (index<0 || index>=colNum)
    {
        return false;
    }
    else
    {
        value = colData[index][colour];
        return true;
    }
}


bool TransferData::getIMU(int index, ImuVec vec, int& value)
{
    if (index<0 || index>=imuNum)
    {
        return false;
    }
    else
    {
        value = imuData[index][vec];
        return true;
    }
    
}

bool TransferData::getRPM(int index, int& value)
{
    if (index<0 || index >=motorNum)
    {
        return false;
    }
    else
    {
        value = rpmData[index];
        return true;
    }

}

bool TransferData::getPos(int index, int& value)
{
    if (index<0 || index >=motorNum)
    {
        return false;
    }
    else
    {
        value = posData[index];
        return true;
    }

}


bool TransferData::setTof(int index, int value)
{
    if (index<0 || index >= tofNum)
    {
        return false;
    }
    else
    {
        tofData[index] = value;
        return true;
    }

}

bool TransferData::setCol(int index, Col colour, int value)
{
    if (index<0 || index>=colNum)
    {
        return false;
    }
    else
    {
        colData[index][colour] = value;
        return true;
    }

}

bool TransferData::setIMU(int index, ImuVec vec, int value)
{
    if (index<0 || index>=imuNum)
    {
        return false;
    }
    else
    {
        imuData[index][vec] = value;
        return true;
    }

}

bool TransferData::setRPM(int index, int value)
{
    if (index<0 || index >=motorNum)
    {
        return false;
    }
    else
    {
        rpmData[index] = value;
        return true;
    }

}

bool TransferData::setPos(int index, int value)
{
    if (index<0 || index >=motorNum)
    {
        return false;
    }
    else
    {
        posData[index] = value;
        return true;
    }
}