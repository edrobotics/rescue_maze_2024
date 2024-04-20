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
        
        std::queue<Victim> statusQueue;

        public:
        void addVictimToStatusQueue(Victim victim); //Fusion should add to this queue
        bool hasNonStatusVictims();
        Victim getNextNonStatusVictim();
        std::vector<Victim> getAllNonStatusVictims();

        void clearVictimsBecauseLOP();
    };
}