#include "communicator/panicFlagCommunicator.h"

namespace communication
{
    PanicFlag::PanicFlag(std::vector<ReadThread> threadDependencies)
    {
        for (std::size_t i = 0; i < threadDependencies.size(); i++)
        {
            if (threadDependencies[i] == ReadThread::globalNav)
                dependencies.push_back(&readByGlobalNav);
            if (threadDependencies[i] == ReadThread::localNav)
                dependencies.push_back(&readByLocalNav);
            if (threadDependencies[i] == ReadThread::fusion)
                dependencies.push_back(&readByFusion);
        }
    }

    void PanicFlagCommunicator::raiseFlag(PanicFlags flag)
    {
        getPanicFlag(flag)->flagIsRaised = true;
    }

    bool PanicFlagCommunicator::readFlagFromThread(PanicFlags flag, ReadThread fromThread)
    {
        PanicFlag* panicFlag = getPanicFlag(flag);
        if (panicFlag->flagIsRaised)
        {
            if (hasBeenSeenByEveryone(panicFlag)) panicFlag->flagIsRaised = false;
            return true;
        }
        
        return false;
    }

    PanicFlag* PanicFlagCommunicator::getPanicFlag(PanicFlags flag)
    {
        switch (flag)
        {
        case PanicFlags::victimDetected:
            return &victimFlag;
        case PanicFlags::onRamp:
            return &rampFlag;
        case PanicFlags::sawBlackTile:
            return &blackFlag;
        default:
            return &lOPFlag;
        }
    }

    bool PanicFlagCommunicator::hasBeenSeenByEveryone(PanicFlag* flag)
    {
        for (std::size_t i = 0; i < flag->dependencies.size(); i++)
        {
            if (! (*(flag->dependencies[i])) )
            {
                return false;
            }
        }
        return true;
    }
}