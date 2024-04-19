#include "fusion/lackOfProgressChecker.h"

void LackOfProgressChecker::loopChecker(communication::Communicator* communicatorInstance, PiAbstractor* piAbstractorInstance)
{
    communicator = communicatorInstance;
    piAbstractor = piAbstractorInstance;
    bool lackOfProgressFlagRaised = false;
    piAbstractor->pinModeTS(LOPSwitchPin, PiAbstractor::PinMode::inputPullup);

    while (true)
    {
        if (lackOfProgressSwitchIsActive())
        {
            if (!lackOfProgressFlagRaised)
            {
                communicator->panicFlagComm.raiseFlag(communication::PanicFlags::lackOfProgressActivated);
                lackOfProgressFlagRaised = true;
            }
        }
        else
        {
            if (lackOfProgressFlagRaised)
            {
                communicator->panicFlagComm.raiseFlag(communication::PanicFlags::lackOfProgressDeactivated);
                lackOfProgressFlagRaised = false;
            }
        }
        std::this_thread::sleep_for(SleepTimeMS);
    }
}

bool LackOfProgressChecker::lackOfProgressSwitchIsActive()
{
    //high = true = normal
    //low = false = LOP
    return !piAbstractor->digitalReadTS(LOPSwitchPin);
}