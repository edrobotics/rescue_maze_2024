#include "communicator/PoseCommunicator.h"

namespace communication
{
    PoseCommunicator::PoseCommunicator()
    {
        
    }

    // PoseCommunicator& PoseCommunicator::operator=(const PoseCommunicator& pComm)
    // {
    //     // std::cout << "Began assignment of PoseCommunicator" << std::endl;

    //     poseDataBlob = pComm.poseDataBlob;

    //     freshness = pComm.freshness;
    //     updated = pComm.updated;

    //     // std::cout << "copying times... ";
    //     // std::cout << "done\n";

    //     mtx_actionControl.lock();
    //     isTurning = pComm.isTurning;
    //     isDriving = pComm.isDriving;
    //     mtx_actionControl.unlock();


    //     shouldflush = pComm.shouldflush;

    //     return *this;
    // }

    // PoseCommunicator::PoseCommunicator(const PoseCommunicator& pComm)
    // {
    //     *this = pComm;
    // }


    bool PoseCommunicator::getTurning()
    {
        std::lock_guard<std::mutex> lock(mtx_actionControl);
        return isTurning;
    }

    void PoseCommunicator::setTurning(bool turning)
    {
        std::lock_guard<std::mutex> lock(mtx_actionControl);
        isTurning = turning;
    }

    bool PoseCommunicator::getDriving()
    {
        std::lock_guard<std::mutex> lock(mtx_actionControl);
        return isDriving;
    }

    void PoseCommunicator::setDriving(bool driving)
    {
        std::lock_guard<std::mutex> lock(mtx_actionControl);
        isDriving = driving;
    }

    double PoseCommunicator::getFrontObstacleDist()
    {
        std::lock_guard<std::mutex> lock(mtx_frontObstacle);
        return frontObstacleDist;
    }

    void PoseCommunicator::setFrontObstacleDist(double dist)
    {
        std::lock_guard<std::mutex> lock(mtx_frontObstacle);
        frontObstacleDist = dist;
    }

    bool PoseCommunicator::hasDrivenStep()
    {
        PoseDataSyncBlob pdBlob {};
        if (pdBlob.getLocalTileFrame().transform==pdBlob.getStartLocalTileFrame().transform)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void PoseCommunicator::flushPose()
    {
        mtx_flush.lock();
        shouldflush = true;
        mtx_flush.unlock();
        bool goOn {true};

        while (goOn)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            mtx_flush.lock();
            goOn = shouldflush;
            mtx_flush.unlock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
    }

    bool PoseCommunicator::getShouldFlushPose()
    {
        std::lock_guard<std::mutex> lock(mtx_flush);
        bool retVal {false};
        if (shouldflush)
        {
            // shouldflush = false;
            return true;
        }
        else
        {
            return shouldflush;
        }
    }

    void PoseCommunicator::flushDone()
    {
        std::lock_guard<std::mutex> lock(mtx_flush);
        shouldflush = false;
    }
}