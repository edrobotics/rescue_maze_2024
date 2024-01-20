#pragma once
#include <thread>
#include <mutex>

#include "TransferData/TransferData.h"
#include "fusion/teensyCommunicator.h"
// #include "fusion/mutexes.h" // Might not be needed

#include "fusion/Sensors.h"
// #include "fusion/Imu.h"
// #include "fusion/Tof.h"
#include "fusion/MotorControllers.h"

namespace fusion
{    
    void main();
}