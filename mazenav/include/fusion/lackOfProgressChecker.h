#pragma once

#include <thread>

#include "communicator/communicator.h"
#include "fusion/PiAbstractor.h"

class LackOfProgressChecker
{
    private:
    const int LOPSwitchPin = 7;
    const std::chrono::milliseconds SleepTimeMS = std::chrono::milliseconds(73);
    bool lackOfProgressSwitchIsActive();
    communication::Communicator* communicator;
    PiAbstractor* piAbstractor;
    
    public:
    void loopChecker(communication::Communicator* communicatorInstance, PiAbstractor* piAbstractorInstance);
};