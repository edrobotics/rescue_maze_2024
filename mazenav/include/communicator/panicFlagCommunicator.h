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
        lackOfProgressActivated,
        lackOfProgressDeactivated,
        droveHalfTile
    };

    struct PanicFlag //IMPORTANT - add new flags to resetAllFlags(), here, as a member in PanicFlagCommunicator and in getPanicFlag
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
        PanicFlag victimFlag{{ ReadThread::localNav, ReadThread::globalNav }};
        PanicFlag rampFlag{{ ReadThread::localNav }};
        PanicFlag blackFlag{{ ReadThread::localNav }};
        PanicFlag lOPFlag{{ ReadThread::globalNav, ReadThread::localNav , ReadThread::fusion}};
        PanicFlag lOPDoneFlag{{ReadThread::globalNav, ReadThread::localNav, ReadThread::fusion}};
        PanicFlag droveHalfTileFlag{{ ReadThread::globalNav }};

        PanicFlag* getPanicFlag(PanicFlags flag);
        bool hasBeenSeenByEveryone(PanicFlag* flag);
        void resetAllFlags();

        public:
        void raiseFlag(PanicFlags flag);
        bool readFlagFromThread(PanicFlags flag, ReadThread fromThread);
    };
}