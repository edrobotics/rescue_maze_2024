#include "fusion/TransferDataWrapper.h"

void TransferDataWrapper::tsCompose()
{
    mtx_transData_freqData.lock();
    mtx_transData_infreqData.lock();
    compose();
    mtx_transData_freqData.unlock();
    mtx_transData_infreqData.unlock();

}
void TransferDataWrapper::tsDecompose()
{
    mtx_transData_freqData.lock();
    mtx_transData_infreqData.lock();
    decompose();
    mtx_transData_freqData.unlock();
    mtx_transData_infreqData.unlock();

}


void TransferDataWrapper::tsComposeSettings()
{
    mtx_transData_controlData.lock();
    composeSettings();
    mtx_transData_controlData.unlock();

}


void TransferDataWrapper::tsDecomposeSettings()
{
    mtx_transData_controlData.lock();
    decomposeSettings();
    mtx_transData_controlData.unlock();

}


void TransferDataWrapper::tsGetByteArr(uint8_t data[])
{
    mtx_transData_freqData.lock();
    mtx_transData_infreqData.lock();
    getByteArr(data);
    mtx_transData_freqData.unlock();
    mtx_transData_infreqData.unlock();

}

void TransferDataWrapper::tsSetByteArr(uint8_t data[])
{
    mtx_transData_freqData.lock();
    mtx_transData_infreqData.lock();
    setByteArr(data);
    mtx_transData_freqData.unlock();
    mtx_transData_infreqData.unlock();

}

void TransferDataWrapper::tsGetControlArr(uint8_t data[])
{
    mtx_transData_controlData.lock();
    getControlArr(data);
    mtx_transData_controlData.unlock();
}

void TransferDataWrapper::tsSetControlArr(uint8_t data[])
{
    mtx_transData_controlData.lock();
    setControlArr(data);
    mtx_transData_controlData.unlock();
}

bool TransferDataWrapper::tsGetTof(uint16_t values[])
{
    mtx_transData_freqData.lock();
    bool ret {getTof(values)};
    mtx_transData_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetCol(int index, uint16_t values[])
{
    mtx_transData_freqData.lock();
    bool ret {getCol(index, values)};
    mtx_transData_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetIMU(int index, float values[])
{
    mtx_transData_freqData.lock();
    bool ret {getIMU(index, values)};
    mtx_transData_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetRPM(int16_t values[])
{
    mtx_transData_freqData.lock();
    bool ret {getRPM(values)};
    mtx_transData_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetPos(int16_t values[])
{
    mtx_transData_freqData.lock();
    bool ret {getPos(values)};
    mtx_transData_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetRpmControl(int16_t values[])
{
    mtx_transData_controlData.lock();
    bool ret {getRpmControl(values)};
    mtx_transData_controlData.unlock();
    return ret;

}


bool TransferDataWrapper::tsSetRpmControl(int16_t values[])
{
    mtx_transData_controlData.lock();
    bool ret {setRpmControl(values)};
    mtx_transData_controlData.unlock();
    return ret;

}

