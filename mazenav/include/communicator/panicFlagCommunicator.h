#pragma once

#include <atomic>
#include <vector>

namespace communication
{
    enum class ReadThread
    {
        globalNav,
        localNav,
        fusion
    };

    enum class PanicFlags
    {
        victimDetected,
        onRamp,
        sawBlackTile,
        lackOfProgressActivated
    };

    struct PanicFlag
    {
        std::atomic_bool flagIsRaised = false;
        std::atomic_bool readByGlobalNav = false;
        std::atomic_bool readByLocalNav = false;
        std::atomic_bool readByFusion = false;

        std::vector<std::atomic_bool*> dependencies;

        PanicFlag(std::vector<ReadThread> threadDependencies);
    };

    class PanicFlagCommunicator
    {
        private:
        PanicFlag victimFlag{{ ReadThread::localNav }};
        PanicFlag rampFlag{{ ReadThread::localNav }};
        PanicFlag blackFlag{{ ReadThread::localNav }};
        PanicFlag lOPFlag{{ ReadThread::globalNav, ReadThread::localNav }};

        PanicFlag* getPanicFlag(PanicFlags flag);
        bool hasBeenSeenByEveryone(PanicFlag* flag);

        std::vector<PanicFlags> getActivePanicFlags(ReadThread readThread);

        public:
        void raiseFlag(PanicFlags flag);
        bool readFlagFromThread(PanicFlags flag, ReadThread fromThread);
    };
}