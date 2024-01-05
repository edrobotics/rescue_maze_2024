#pragma once
#include "Bno085.h"

class Imu
{
    public:
        Imu();
        bool init();

        bool runLoop();

        Quaternion rotationVector {};

    private:
        int i2cAddr0 = 0x4a; // Default address of bno085. 0x4b can be used with hardware modification
        Bno085 bno085 {i2cAddr0};
};