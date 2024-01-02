// Idea: Either do byte array and copy into it, or make a byte pointer
// Other idea: Use normal datatypes first, but store how many bytes they are allowed to use. Then iterate through them to create the byte array and then iterate backwards to get values again.

#pragma once

#include <stdint.h> // Types
#include <cstring> // memcpy
// #include <iostream>

class TransferData
{
    public:
    // Default constructor
    TransferData();
    // Compose the int into the array
    void compose();
    // Decompose the array into the int
    void decompose();

    void test();
    void composeSettings();

    void decomposeSettings();

    void getByteArr(uint8_t data[]);
    void getControlArr(uint8_t data[]);

    static const int dataLen = 64; // Data length in bytes
    static const int settingLen = 8;
    uint8_t byteArr[dataLen] {0};
    uint8_t controlArr[settingLen] {0};

    enum Col
    {
        col_r,
        col_g,
        col_b,
        col_c,
        col_num,
    };

    enum ImuVec
    {
        imu_x,
        imu_y,
        imu_z,
        imu_num,
    };
    
    // Getters
    bool getTof(int index, int& value);
    bool getCol(int index, Col colour, int& value);
    bool getIMU(int index, ImuVec vec, int& value);
    bool getRPM(int index, int& value);
    bool getPos(int index, int& value);

    // Setters for teensy-pi
    bool setTof(int index, int value);
    bool setCol(int index, Col colour, int value);
    bool setIMU(int index, ImuVec vec, int value);
    bool setRPM(int index, int value);
    bool setPos(int index, int value);

    // Setters for pi-teensy
    bool setRpmControl(int index, int value);
    bool getRpmControl(int index, int& value);


    private:
        // Teensy to pi

        static const int tofNum = 7;
        uint16_t tofData[tofNum] {0};

        static const int colNum = 2;
        uint16_t colData[colNum][col_num] {0};

        static const int imuNum = 3;
        int16_t imuData[imuNum][imu_num] {0};

        static const int motorNum = 4;
        int16_t rpmData[motorNum] {0};
        int16_t posData[motorNum] {0};


        // Pi to teensy
        int16_t rpmControlVals[motorNum] {0};


    void u16toB(uint16_t input, uint8_t& byte1, uint8_t& byte2);
    void s16toB(int16_t input, uint8_t& byte1, uint8_t& byte2);
    void btoU16(uint8_t input1, uint8_t input2, uint16_t& output);
    void btoS16(uint8_t input1, uint8_t input2, int16_t& output);
};