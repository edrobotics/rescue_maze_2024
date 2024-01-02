#pragma once

#include <stdint.h> // Types
#include <cstring> // memcpy

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

    void getByteArr(uint8_t data[]);


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

    // Setters
    bool setTof(int index, int value);
    bool setCol(int index, Col colour, int value);
    bool setIMU(int index, ImuVec vec, int value);
    bool setRPM(int index, int value);
    bool setPos(int index, int value);


    private:
    static const int dataLen = 64; // Data length in bytes

    static const int tofNum = 7;
    uint16_t tofData[tofNum] {};

    static const int colNum = 2;
    uint16_t colData[colNum][col_num] {};

    static const int imuNum = 3;
    int16_t imuData[imuNum][imu_num] {};

    static const int motorNum = 4;
    int16_t rpmData[motorNum] {};
    int16_t posData[motorNum] {};

    uint8_t byteArr[dataLen] {};

    void u16toB(uint16_t input, uint8_t& byte1, uint8_t& byte2);
    void s16toB(int16_t input, uint8_t& byte1, uint8_t& byte2);
    void btoU16(uint8_t input1, uint8_t input2, uint16_t& output);
    void btoS16(uint8_t input1, uint8_t input2, int16_t& output);
};