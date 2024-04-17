#pragma once

#include <vector>
#include <queue>
#include <mutex>
#include "fusion/Victim.h"

namespace communication
{
    class VictimDataCommunicator
    {
        private:
        std::mutex mtx_status;
        std::mutex mtx_rescue;
        
        std::queue<Victim> statusQueue;
        std::queue<Victim> rescueQueue;

        public:
        void addVictimToStatusQueue(Victim victim); //Fusion should add to this queue
        bool hasNonStatusVictims();
        Victim getNextNonStatusVictim();
        std::vector<Victim> getAllNonStatusVictims();

        void addVictimToRescueQueue(Victim victim);
        bool hasUnrescuedVictims();
        Victim getNextUnrescuedVictim();
        std::vector<Victim> getAllUnrescuedVictims(); //LocalNav should read this or the above function (but this one is less locking)
    };
}