#include "fusion/TransferData.h"


TransferData::TransferData()
{

}

void TransferData::compose()
{
    for (int i=0;i<dataLen;++i)
    {
        int shiftNum = 8*(dataLen-i-1);
        byteArr[i] = static_cast<uint8_t>((dataInt & (0b11111111 << shiftNum)) >> shiftNum);
    }

}

void TransferData::decompose()
{
    dataInt = 0;
    for (int i=0;i<dataLen;++i)
    {
        int shiftNum = 8*(dataLen-i-1);
        dataInt |= byteArr[i] << shiftNum;
    }
}

unsigned int TransferData::getTof(int index)
{
    switch (index)
    {
        case 0:
            return static_cast<unsigned int>((dataInt & MSK_TOF0) >> TOF_SHIFT0);
            break;
        case 1:
            return static_cast<unsigned int>((dataInt & MSK_TOF1) >> TOF_SHIFT1);
            break;
        case 2:
            return static_cast<unsigned int>((dataInt & MSK_TOF2) >> TOF_SHIFT2);
            break;
        case 3:
            return static_cast<unsigned int>((dataInt & MSK_TOF3) >> TOF_SHIFT3);
            break;
        case 4:
            return static_cast<unsigned int>((dataInt & MSK_TOF4) >> TOF_SHIFT4);
            break;
        case 5:
            return static_cast<unsigned int>((dataInt & MSK_TOF5) >> TOF_SHIFT5);
            break;
        case 6:
            return static_cast<unsigned int>((dataInt & MSK_TOF6) >> TOF_SHIFT6);
            break;
        case 7:
            return static_cast<unsigned int>((dataInt & MSK_TOF7) >> TOF_SHIFT7);
            break;
        case 8:
            return static_cast<unsigned int>((dataInt & MSK_TOF8) >> TOF_SHIFT8);
            break;
        case 9:
            return static_cast<unsigned int>((dataInt & MSK_TOF9) >> TOF_SHIFT9);
            break;
        default:
            return 0;
            break;
    }
}

unsigned int TransferData::getCol(int index, Col colour)
{
    switch (index)
    {
        case 0:
            switch(colour)
            {
                case col_r:
                    return static_cast<unsigned int>((dataInt & MSK_COL0R) >> COL_SHIFT0R);
                    break;
                case col_g:
                    return static_cast<unsigned int>((dataInt & MSK_COL0G) >> COL_SHIFT0G);
                    break;
                case col_b:
                    return static_cast<unsigned int>((dataInt & MSK_COL0B) >> COL_SHIFT0B);
                    break;
                case col_c:
                    return static_cast<unsigned int>((dataInt & MSK_COL0C) >> COL_SHIFT0C);
                    break;
                default:
                    return 0;
                    break;
            }
        case 1:
            switch(colour)
            {
                case col_r:
                    return static_cast<unsigned int>((dataInt & MSK_COL1R) >> COL_SHIFT1R);
                    break;
                case col_g:
                    return static_cast<unsigned int>((dataInt & MSK_COL1G) >> COL_SHIFT1G);
                    break;
                case col_b:
                    return static_cast<unsigned int>((dataInt & MSK_COL1B) >> COL_SHIFT1B);
                    break;
                case col_c:
                    return static_cast<unsigned int>((dataInt & MSK_COL1C) >> COL_SHIFT1C);
                    break;
                default:
                    return 0;
                    break;
            }
        default:
            return 0;
            break;

    }
}


void TransferData::getIMU(int index)
{
    
}

int TransferData::getRPM(int index)
{
    switch (index)
    {
        case 0:
            return static_cast<int>((dataInt & MSK_RPM0) >> RPM_SHIFT0);
            break;
        case 1:
            return static_cast<int>((dataInt & MSK_RPM1) >> RPM_SHIFT1);
            break;
        case 2:
            return static_cast<int>((dataInt & MSK_RPM2) >> RPM_SHIFT2);
            break;
        case 3:
            return static_cast<int>((dataInt & MSK_RPM3) >> RPM_SHIFT3);
            break;
        default:
            return 0;
            break;
    }

}

int TransferData::getPos(int index)
{
    switch (index)
    {
        case 0:
            return static_cast<int>((dataInt & MSK_POS0) >> POS_SHIFT0);
            break;
        case 1:
            return static_cast<int>((dataInt & MSK_POS1) >> POS_SHIFT1);
            break;
        case 2:
            return static_cast<int>((dataInt & MSK_POS2) >> POS_SHIFT2);
            break;
        case 3:
            return static_cast<int>((dataInt & MSK_POS3) >> POS_SHIFT3);
            break;
        default:
            return 0;
            break;
    }

}


void TransferData::setTof(int index, int value)
{
    switch (index)
    {
        case 0:
            dataInt &= ~MSK_TOF1;
            dataInt |= MSK_TOF1 & value;
            break;
        default:
            break;
    }


}

void TransferData::setCol(int index, Col colour, int value)
{

}

void TransferData::setIMU(int index, int value)
{

}

void TransferData::setRPM(int index, int value)
{

}

void TransferData::setPos(int index, int value)
{

}

void TransferData::printInt()
{
    std::cout << "dataInt=" << "\n" << std::hex << dataInt << "\n";
}