#pragma once
#include <thread>
#include <mutex>

#include "TransferData/TransferData.h" // Should ideally not be needed here
#include "fusion/teensyCommunicator.h" // For communication with teensy
#include "communicator/communicator.h" // For global communication
// #include "fusion/mutexes.h" // Might not be needed

#include "fusion/Sensors.h"
// #include "fusion/Imu.h"
// #include "fusion/Tof.h"
#include "fusion/MotorControllers.h"

namespace fusion
{    
    void main(communication::Communicator* globComm);
}