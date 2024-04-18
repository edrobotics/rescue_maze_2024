#pragma once

#include "GlobalConstants.h"
#include "communicator/logger.h"
#include "communicator/timer.h"

#include "communicator/MotorControllerCommunicator.h"
#include "communicator/PoseCommunicator.h"
#include "communicator/navigationCommunicator.h"
#include "communicator/tileDrivePropertyCommunicator.h"
#include "communicator/panicFlagCommunicator.h"
#include "communicator/victimDataCommunicator.h"

namespace communication
{
    // Class containing the data that we want to share
    class Communicator
    {
        public:
        NavigationCommunicator navigationComm;
        PoseCommunicator poseComm {};
        MotorControllerCommunicator motors{};
        TileDrivePropertyCommunicator tileInfoComm;
        PanicFlagCommunicator panicFlagComm;
        VictimDataCommunicator victimDataComm{};
        Logger logger {};
        Timer timer {};
    };
}