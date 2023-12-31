// Idea: Either do byte array and copy into it, or make a byte pointer

// Other idea: Use normal datatypes first, but store how many bytes they are allowed to use. Then iterate through them to create the byte array and then iterate backwards to get values again.


#pragma once
#include <boost/multiprecision/cpp_int.hpp>
#include <bitset>

using namespace boost::multiprecision;

// BITMASKS

// TOF
#define TOF_SIZE 10
#define TOF_STARTSHIFT 512-TOF_SIZE*1

#define TOF_SHIFT0 TOF_STARTSHIFT-TOF_SIZE*0
#define TOF_SHIFT1 TOF_STARTSHIFT-TOF_SIZE*1
#define TOF_SHIFT2 TOF_STARTSHIFT-TOF_SIZE*2
#define TOF_SHIFT3 TOF_STARTSHIFT-TOF_SIZE*3
#define TOF_SHIFT4 TOF_STARTSHIFT-TOF_SIZE*4
#define TOF_SHIFT5 TOF_STARTSHIFT-TOF_SIZE*5
#define TOF_SHIFT6 TOF_STARTSHIFT-TOF_SIZE*6
#define TOF_SHIFT7 TOF_STARTSHIFT-TOF_SIZE*7
#define TOF_SHIFT8 TOF_STARTSHIFT-TOF_SIZE*8
#define TOF_SHIFT9 TOF_STARTSHIFT-TOF_SIZE*9

#define MSK_TOF0 0b1111111111 << TOF_SHIFT0
#define MSK_TOF1 0b1111111111 << TOF_SHIFT1
#define MSK_TOF2 0b1111111111 << TOF_SHIFT2
#define MSK_TOF3 0b1111111111 << TOF_SHIFT3
#define MSK_TOF4 0b1111111111 << TOF_SHIFT4
#define MSK_TOF5 0b1111111111 << TOF_SHIFT5
#define MSK_TOF6 0b1111111111 << TOF_SHIFT6
#define MSK_TOF7 0b1111111111 << TOF_SHIFT7
#define MSK_TOF8 0b1111111111 << TOF_SHIFT8
#define MSK_TOF9 0b1111111111 << TOF_SHIFT9

#define TOF_ENDSHIFT TOF_SHIFT9


// Colour sensors
#define COL_SIZE  14
#define COL_STARTSHIFT TOF_ENDSHIFT - COL_SIZE*1

#define COL_SHIFT0R COL_STARTSHIFT-COL_SIZE*0
#define COL_SHIFT0G COL_STARTSHIFT-COL_SIZE*1
#define COL_SHIFT0B COL_STARTSHIFT-COL_SIZE*2
#define COL_SHIFT0C COL_STARTSHIFT-COL_SIZE*3
#define COL_SHIFT1R COL_STARTSHIFT-COL_SIZE*4
#define COL_SHIFT1G COL_STARTSHIFT-COL_SIZE*5
#define COL_SHIFT1B COL_STARTSHIFT-COL_SIZE*6
#define COL_SHIFT1C COL_STARTSHIFT-COL_SIZE*7

#define MSK_COL0R 0b11111111111111 << COL_SHIFT0R
#define MSK_COL0G 0b11111111111111 << COL_SHIFT0G
#define MSK_COL0B 0b11111111111111 << COL_SHIFT0B
#define MSK_COL0C 0b11111111111111 << COL_SHIFT0C
#define MSK_COL1R 0b11111111111111 << COL_SHIFT1R
#define MSK_COL1G 0b11111111111111 << COL_SHIFT1G
#define MSK_COL1B 0b11111111111111 << COL_SHIFT1B
#define MSK_COL1C 0b11111111111111 << COL_SHIFT1C

#define COL_ENDSHIFT COL_SHIFT1C


// IMU
#define IMU_SIZE 14
#define IMU_STARTSHIFT COL_ENDSHIFT - IMU_SIZE

#define IMU_SHIFT00 IMU_STARTSHIFT-IMU_SIZE*0
#define IMU_SHIFT01 IMU_STARTSHIFT-IMU_SIZE*1
#define IMU_SHIFT02 IMU_STARTSHIFT-IMU_SIZE*2
#define IMU_SHIFT10 IMU_STARTSHIFT-IMU_SIZE*3
#define IMU_SHIFT11 IMU_STARTSHIFT-IMU_SIZE*4
#define IMU_SHIFT12 IMU_STARTSHIFT-IMU_SIZE*5
#define IMU_SHIFT20 IMU_STARTSHIFT-IMU_SIZE*6
#define IMU_SHIFT21 IMU_STARTSHIFT-IMU_SIZE*7
#define IMU_SHIFT22 IMU_STARTSHIFT-IMU_SIZE*8

#define MSK_IMU00 0b11111111111111 << IMU_SHIFT00
#define MSK_IMU01 0b11111111111111 << IMU_SHIFT01
#define MSK_IMU02 0b11111111111111 << IMU_SHIFT02

#define MSK_IMU10 0b11111111111111 << IMU_SHIFT10
#define MSK_IMU11 0b11111111111111 << IMU_SHIFT11
#define MSK_IMU12 0b11111111111111 << IMU_SHIFT12

#define MSK_IMU20 0b11111111111111 << IMU_SHIFT20
#define MSK_IMU21 0b11111111111111 << IMU_SHIFT21
#define MSK_IMU22 0b11111111111111 << IMU_SHIFT22

#define IMU_ENDSHIFT IMU_SHIFT22


// RPM
#define RPM_SIZE 9
#define RPM_STARTSHIFT IMU_ENDSHIFT-RPM_SIZE

#define RPM_SHIFT0 RPM_STARTSHIFT-RPM_SIZE*0
#define RPM_SHIFT1 RPM_STARTSHIFT-RPM_SIZE*1
#define RPM_SHIFT2 RPM_STARTSHIFT-RPM_SIZE*2
#define RPM_SHIFT3 RPM_STARTSHIFT-RPM_SIZE*3

#define MSK_RPM0 0b111111111 << RPM_SHIFT0
#define MSK_RPM1 0b111111111 << RPM_SHIFT1
#define MSK_RPM2 0b111111111 << RPM_SHIFT2
#define MSK_RPM3 0b111111111 << RPM_SHIFT3

#define RPM_ENDSHIFT RPM_SHIFT3


//POS
#define POS_SIZE 16
#define POS_STARTSHIFT RPM_ENDSHIFT-POS_SIZE

#define POS_SHIFT0 POS_STARTSHIFT-POS_SIZE*0
#define POS_SHIFT1 POS_STARTSHIFT-POS_SIZE*1
#define POS_SHIFT2 POS_STARTSHIFT-POS_SIZE*2
#define POS_SHIFT3 POS_STARTSHIFT-POS_SIZE*3

#define MSK_POS0 0b1111111111111111 << POS_SHIFT0
#define MSK_POS1 0b1111111111111111 << POS_SHIFT1
#define MSK_POS2 0b1111111111111111 << POS_SHIFT2
#define MSK_POS3 0b1111111111111111 << POS_SHIFT3

#define POS_ENDSHIFT POS_SHIFT3

// END BITMASKS

class TransferData
{
    public:
    // Default constructor
    TransferData();
    // Compose the int into the array
    void compose();
    // Decompose the array into the int
    void decompose();
    // Prints the integer in byte form
    void printInt();
    // Prints the byte array in byte form
    void printArr();

    enum Col
    {
        col_r,
        col_g,
        col_b,
        col_c,
    };
    
    // Getters
    unsigned int getTof(int index);
    unsigned int getCol(int index, Col colour);
    void getIMU(int index);
    int getRPM(int index);
    int getPos(int index);

    // Setters
    void setTof(int index, int value);
    void setCol(int index, Col colour, int value);
    void setIMU(int index, int value);
    void setRPM(int index, int value);
    void setPos(int index, int value);


    private:
    static const int dataLen = 64; // Data length in bytes
    uint512_t dataInt = 0;
    uint8_t byteArr[dataLen];

    

};