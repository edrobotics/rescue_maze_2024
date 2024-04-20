#include "communicator/victimDataCommunicator.h"

namespace communication
{
    void VictimDataCommunicator::addVictimToStatusQueue(Victim victim)
    {
        mtx_status.lock();

        statusQueue.push(victim);

        mtx_status.unlock();
    }

    bool VictimDataCommunicator::hasNonStatusVictims()
    {
        mtx_status.lock();

        bool hasVictimWithNoStatus = !statusQueue.empty();

        mtx_status.unlock();

        return hasVictimWithNoStatus;
    }

    Victim VictimDataCommunicator::getNextNonStatusVictim()
    {
        mtx_status.lock();

        Victim nextVictim = statusQueue.front();
        statusQueue.pop();

        mtx_status.unlock();

        return nextVictim;
    }

    std::vector<Victim> VictimDataCommunicator::getAllNonStatusVictims()
    {
        std::vector<Victim> unstatusedVictims;
        
        mtx_status.lock();

        while (!statusQueue.empty())
        {
            unstatusedVictims.push_back(statusQueue.front());
            statusQueue.pop();
        }

        mtx_status.unlock();

        return unstatusedVictims;
    }

    void VictimDataCommunicator::clearVictimsBecauseLOP()
    {
        while (!statusQueue.empty())
            statusQueue.pop();
    }

}