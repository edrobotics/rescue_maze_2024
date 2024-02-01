#pragma once
#include "TransferData/TransferData.h"
#include "fusion/mutexes.h"

class TransferDataWrapper : private TransferData
{
    public:

        // ts = thread safe

        // Compose the int into the array
        void tsCompose();
        // Decompose the array into the int
        void tsDecompose();

        void tsComposeSettings();

        void tsDecomposeSettings();

        void tsGetByteArr(uint8_t data[]);
        void tsSetByteArr(uint8_t data[]);
        void tsGetControlArr(uint8_t data[]);
        void tsSetControlArr(uint8_t data[]);

        static const int W_DATA_LEN = DATA_LEN; // Data length in bytes
        static const int W_SETTING_LEN = SETTING_LEN;

        // Getters
        bool tsGetTof(uint16_t values[]);
        bool tsGetCol(int index, uint16_t values[]);
        bool tsGetIMU(int index, float values[]);
        bool tsGetRPM(int16_t values[]);
        bool tsGetPos(int16_t values[]);
        bool tsGetRpmControl(int16_t values[]);

        bool tsSetRpmControl(int16_t values[]);
    
    private:
    std::mutex mtx_freqData;
    std::mutex mtx_controlData;
    std::mutex mtx_infreqData;
};