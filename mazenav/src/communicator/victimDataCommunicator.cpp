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

    void VictimDataCommunicator::addVictimToRescueQueue(Victim victim)
    {
        mtx_rescue.lock();

        rescueQueue.push(victim);

        mtx_rescue.unlock();
    }

    bool VictimDataCommunicator::hasUnrescuedVictims()
    {
        mtx_rescue.lock();

        bool hasVictim = !rescueQueue.empty();

        mtx_rescue.unlock();

        return hasVictim;
    }

    Victim VictimDataCommunicator::getNextUnrescuedVictim()
    {
        mtx_rescue.lock();

        Victim nextVictim = rescueQueue.front();
        rescueQueue.pop();

        mtx_rescue.unlock();

        return nextVictim;
    }

    std::vector<Victim> VictimDataCommunicator::getAllUnrescuedVictims()
    {
        std::vector<Victim> unrescuedVictims;
        
        mtx_rescue.lock();

        while (!rescueQueue.empty())
        {
            unrescuedVictims.push_back(rescueQueue.front());
            rescueQueue.pop();
        }

        mtx_rescue.unlock();

        return unrescuedVictims;
    }
}