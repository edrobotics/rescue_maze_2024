#pragma once
#include <thread>
#include <mutex>
#include <chrono>

// #include "TransferData/TransferData.h" // Should ideally not be needed here
#include "fusion/TeensyCommunicator.h" // For communication with teensy
#include "communicator/communicator.h" // For global communication
// #include "fusion/mutexes.h" // Might not be needed

#include "fusion/Sensors.h"
#include "fusion/MotorControllers.h"
#include "fusion/PoseEstimator.h"

#include "fusion/LedControl.h"

namespace fusion
{    
    void main(communication::Communicator* globComm);

    void motorDriveLoop(communication::Communicator* gCom, MotorControllers* mot);
    void motorDriveLoopLooper(communication::Communicator* gCom, MotorControllers* mot);
}