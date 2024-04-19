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
        for (auto& dependency : getPanicFlag(flag)->dependencies)
        {
            *dependency = false;
        }
    }

    bool PanicFlagCommunicator::readFlagFromThread(PanicFlags flag, ReadThread fromThread)
    {
        PanicFlag* panicFlag = getPanicFlag(flag);

        switch (fromThread)
        {
            case ReadThread::fusion:
                panicFlag->readByFusion = true;
                break;
            case ReadThread::localNav:
                panicFlag->readByLocalNav = true;
                break;
            case ReadThread::globalNav:
                panicFlag->readByFusion = true;
                break;
            default:
                break;
        }

        if (panicFlag->flagIsRaised)
        {
            if (hasBeenSeenByEveryone(panicFlag)) 
            {
                panicFlag->flagIsRaised = false;

                if (flag == PanicFlags::lackOfProgressDeactivated)
                    resetAllFlags();
            }
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
        case PanicFlags::lackOfProgressDeactivated:
            return &lOPDoneFlag;
        case PanicFlags::lackOfProgressActivated:
            return &lOPFlag;
        default:
            return &droveHalfTileFlag;
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

    void PanicFlagCommunicator::resetAllFlags()
    {
        victimFlag.flagIsRaised = false;
        rampFlag.flagIsRaised = false;
        blackFlag.flagIsRaised = false;
        lOPFlag.flagIsRaised = false;
        lOPDoneFlag.flagIsRaised = false;
        droveHalfTileFlag.flagIsRaised = false;
    }
}