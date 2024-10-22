#include "TransferData.h"

TransferData::TransferData()
{

}

void TransferData::compose()
{
    int arrIdx = 0;
    // ToF
    for (int i=0;i<TOF_NUM;++i)
    {
        u16toB(tofData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

    // Colour sensors
    for (int i=0;i<COL_NUM;++i)
    {
        for (int k=0;k<col_num;++k)
        {
            u16toB(colData[i][k], byteArr[arrIdx], byteArr[arrIdx+1]);
            arrIdx+=2;
        }
    }

    // IMU
    for (int i=0;i<IMU_NUM;++i)
    {
        for (int k=0;k<imu_num;++k)
        {
            // Convert from float to int16_t and then to bytes
            s16toB(floatToInt(imuData[i][k]), byteArr[arrIdx], byteArr[arrIdx+1]);
            arrIdx+=2;
        }

    }

    //RPM
    for (int i=0;i<MOTOR_NUM;++i)
    {
        s16toB(rpmData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

    //Pos
    for (int i=0;i<MOTOR_NUM;++i)
    {
        s16toB(posData[i], byteArr[arrIdx], byteArr[arrIdx+1]);
        arrIdx+=2;
    }

}

void TransferData::decompose()
{
    int arrIdx = 0;
    // ToF
    for (int i=0;i<TOF_NUM;++i)
    {
        btoU16(byteArr[arrIdx], byteArr[arrIdx+1], tofData[i]);
        arrIdx+=2;
    }

    // Colour sensors
    for (int i=0;i<COL_NUM;++i)
    {
        for (int k=0;k<col_num;++k)
        {
            btoU16(byteArr[arrIdx], byteArr[arrIdx+1], colData[i][k]);
            arrIdx+=2;
        }
    }

    // IMU
    for (int i=0;i<IMU_NUM;++i)
    {
        for (int k=0;k<imu_num;++k)
        {
            // btof32(byteArr[arrIdx], byteArr[arrIdx+1], byteArr[arrIdx+2], byteArr[arrIdx+3], imuData[i][k]);
            int16_t intVal {0};
            btoS16(byteArr[arrIdx], byteArr[arrIdx+1], intVal);
            imuData[i][k] = intToFloat(intVal);
            arrIdx+=2;
        }

    }

    //RPM
    for (int i=0;i<MOTOR_NUM;++i)
    {
        btoS16(byteArr[arrIdx], byteArr[arrIdx+1], rpmData[i]);
        arrIdx+=2;
    }

    //Pos
    for (int i=0;i<MOTOR_NUM;++i)
    {
        btoS16(byteArr[arrIdx], byteArr[arrIdx+1], posData[i]);
        arrIdx+=2;
    }

}

void TransferData::composeSettings()
{
    int arrIdx = 0;
    for (int i=0;i<MOTOR_NUM;++i)
    {
        s16toB(rpmControlVals[i], controlArr[arrIdx], controlArr[arrIdx+1]);
        arrIdx+=2;
    }

}

void TransferData::decomposeSettings()
{
    int arrIdx = 0;
    for (int i=0;i<MOTOR_NUM;++i)
    {
        btoS16(controlArr[arrIdx], controlArr[arrIdx+1], rpmControlVals[i]);
        arrIdx+=2;
    }

}

void TransferData::u16toB(uint16_t input, uint8_t& byte1, uint8_t& byte2)
{
    byte1 = static_cast<uint8_t>((input & (0b11111111 << 8)) >> 8);
    byte2 = static_cast<uint8_t>(input & 0b11111111);
}

void TransferData::btoU16(uint8_t input1, uint8_t input2, uint16_t& output)
{
    output = static_cast<uint16_t>(input2);
    output |= input1 << 8;
}

void TransferData::s16toB(int16_t input, uint8_t& byte1, uint8_t& byte2)
{
    byte1 = static_cast<uint8_t>((input & (0b11111111 << 8)) >> 8);
    byte2 = static_cast<uint8_t>(input & 0b11111111);
}

void TransferData::btoS16(uint8_t input1, uint8_t input2, int16_t& output)
{
    output = static_cast<uint16_t>(input2);
    output |= input1 << 8;
}

int16_t TransferData::floatToInt(float f)
{
    // Drop remaining decimals (should be okay) (cheaper than rounding?)
    return static_cast<int16_t>(f*multiplier);
}

float TransferData::intToFloat(int16_t i)
{
    return static_cast<float>(i)/static_cast<float>(multiplier);
}

void TransferData::test()
{
    // uint16_t u16 = 5555;
    // int16_t s16 = -5555;
    // uint8_t byte1, byte2, byte3, byte4;
    // s16toB(s16, byte1, byte2);
    // btoS16(byte1, byte2, s16);

}


void TransferData::getByteArr(uint8_t data[])
{
    memcpy(data, byteArr, DATA_LEN);
}

void TransferData::setByteArr(uint8_t data[])
{
    memcpy(byteArr, data, DATA_LEN);
}

void TransferData::getControlArr(uint8_t data[])
{
    memcpy(data, controlArr, SETTING_LEN);
}

void TransferData::setControlArr(uint8_t data[])
{
    memcpy(controlArr, data, SETTING_LEN);
}


bool TransferData::getTof(uint16_t values[])
{
    memcpy(values, tofData, sizeof(tofData[0])*TOF_NUM);
    return true;
}

bool TransferData::getCol(int index, uint16_t values[])
{
    if (index<0 || index>=COL_NUM)
    {
        return false;
    }
    else
    {
        memcpy(values, colData[index], sizeof(colData[index][0])*col_num);
        return true;
    }
}


bool TransferData::getIMU(int index, float values[])
{
    if (index<0 || index>=IMU_NUM)
    {
        return false;
    }
    else
    {
        memcpy(values, imuData[index], sizeof(imuData[index][0])*imu_num);
        return true;
    }
    
}

bool TransferData::getRPM(int16_t values[])
{
    memcpy(values, rpmData, sizeof(rpmData[0])*MOTOR_NUM);
    return true;

}

bool TransferData::getPos(int16_t values[])
{
    memcpy (values, posData, sizeof(posData[0])*MOTOR_NUM);
    return true;

}


bool TransferData::setTof(uint16_t values[])
{
    memcpy(tofData, values, sizeof(tofData[0])*TOF_NUM);
    return true;

}

bool TransferData::setCol(int index, uint16_t values[])
{
    if (index<0 || index>=COL_NUM)
    {
        return false;
    }
    else
    {
        memcpy(colData[index], values, sizeof(colData[index][0])*col_num);
        return true;
    }

}

bool TransferData::setIMU(int index, float values[])
{
    if (index<0 || index>=IMU_NUM)
    {
        return false;
    }
    else
    {
        memcpy(imuData[index], values, sizeof(imuData[index][0])*imu_num);
        return true;
    }

}

bool TransferData::setRPM(int16_t values[])
{
    memcpy(rpmData, values, sizeof(rpmData[0])*MOTOR_NUM);
    return true;
}

bool TransferData::setPos(int16_t values[])
{
    memcpy(posData, values, sizeof(posData[0])*MOTOR_NUM);
    return true;
}

bool TransferData::setRpmControl(int16_t values[])
{
    memcpy(rpmControlVals, values, sizeof(rpmControlVals[0])*MOTOR_NUM);
    return true;
}

bool TransferData::setVoltage(float value)
{
    voltageData = value;
    return true;
}

bool TransferData::getRpmControl(int16_t values[])
{
    memcpy(values, rpmControlVals, sizeof(rpmControlVals[0])*MOTOR_NUM);
    return true;
}