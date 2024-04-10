#include "fusion/TransferDataWrapper.h"

void TransferDataWrapper::tsCompose()
{
    mtx_freqData.lock();
    mtx_infreqData.lock();
    compose();
    mtx_freqData.unlock();
    mtx_infreqData.unlock();

}
void TransferDataWrapper::tsDecompose()
{
    mtx_freqData.lock();
    mtx_infreqData.lock();
    decompose();
    mtx_freqData.unlock();
    mtx_infreqData.unlock();

}


void TransferDataWrapper::tsComposeSettings()
{
    mtx_controlData.lock();
    composeSettings();
    mtx_controlData.unlock();

}


void TransferDataWrapper::tsDecomposeSettings()
{
    mtx_controlData.lock();
    decomposeSettings();
    mtx_controlData.unlock();

}


void TransferDataWrapper::tsGetByteArr(uint8_t data[])
{
    mtx_freqData.lock();
    mtx_infreqData.lock();
    getByteArr(data);
    mtx_freqData.unlock();
    mtx_infreqData.unlock();

}

void TransferDataWrapper::tsSetByteArr(uint8_t data[])
{
    mtx_freqData.lock();
    mtx_infreqData.lock();
    setByteArr(data);
    mtx_freqData.unlock();
    mtx_infreqData.unlock();

}

void TransferDataWrapper::tsGetControlArr(uint8_t data[])
{
    mtx_controlData.lock();
    getControlArr(data);
    mtx_controlData.unlock();
}

void TransferDataWrapper::tsSetControlArr(uint8_t data[])
{
    mtx_controlData.lock();
    setControlArr(data);
    mtx_controlData.unlock();
}

bool TransferDataWrapper::tsGetTof(uint16_t values[])
{
    mtx_freqData.lock();
    bool ret {tofUpdated};
    getTof(values);
    tofUpdated = false;
    mtx_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetCol(int index, uint16_t values[])
{
    mtx_freqData.lock();
    bool ret {colUpdated};
    getCol(index, values);
    colUpdated = false;
    mtx_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetIMU(int index, float values[])
{
    mtx_freqData.lock();
    bool ret {imuUpdated};
    getIMU(index, values);
    imuUpdated = false;
    mtx_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetRPM(int16_t values[])
{
    mtx_freqData.lock();
    bool ret {rpmUpdated};
    getRPM(values);
    rpmUpdated = false;
    mtx_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetPos(int16_t values[])
{
    mtx_freqData.lock();
    bool ret {posUpdated};
    getPos(values);
    posUpdated = false;
    mtx_freqData.unlock();
    return ret;

}


bool TransferDataWrapper::tsGetRpmControl(int16_t values[])
{
    mtx_controlData.lock();
    bool ret {getRpmControl(values)};
    mtx_controlData.unlock();
    return ret;

}


bool TransferDataWrapper::tsSetRpmControl(int16_t values[])
{
    mtx_controlData.lock();
    bool ret {setRpmControl(values)};
    mtx_controlData.unlock();
    return ret;

}

void TransferDataWrapper::setAllUpdated()
{
    tofUpdated = true;
    colUpdated = true;
    imuUpdated = true;
    rpmUpdated = true;
    posUpdated = true;
}