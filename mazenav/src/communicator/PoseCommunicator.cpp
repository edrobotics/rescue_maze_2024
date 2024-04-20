#include "communicator/PoseCommunicator.h"

namespace communication
{
    PoseCommunicator::PoseCommunicator()
    {
        
    }

    PoseCommunicator& PoseCommunicator::operator=(const PoseCommunicator& pComm)
    {
        // std::cout << "Began assignment of PoseCommunicator" << std::endl;

        poseDataBlob = pComm.poseDataBlob;

        updated = pComm.updated;

        // std::cout << "copying times... ";
        // std::cout << "done\n";

        mtx_actionControl.lock();
        isTurning = pComm.isTurning;
        isDriving = pComm.isDriving;
        mtx_actionControl.unlock();


        shouldflush = pComm.shouldflush;

        return *this;
    }

    PoseCommunicator::PoseCommunicator(const PoseCommunicator& pComm)
    {
        *this = pComm;
    }


    PoseDataSyncBlob PoseCommunicator::borrowData()
    {
        return poseDataBlob.borrow();
    }

    PoseDataSyncBlob PoseCommunicator::copyData(bool markAsRead)
    {
        if (markAsRead)
        {
            mtx_updated.lock();
            updated = false;
            mtx_updated.unlock();
        }
        return poseDataBlob.getCopy();
    }

    void PoseCommunicator::giveBackData(PoseDataSyncBlob pdBlob, bool markUpdated)
    {
        poseDataBlob.giveBack(pdBlob);
        if (markUpdated)
        {
            mtx_updated.lock();
            updated = true;
            mtx_updated.unlock();
        }
    }

    void PoseCommunicator::giveBackDummyData()
    {
        poseDataBlob.giveBackDummyData();
    }

    bool PoseCommunicator::getUpdated()
    {
        std::lock_guard<std::mutex> lock(mtx_updated);
        return updated;
    }

    
    void PoseCommunicator::setTargetFrameTransformTS(Transform tf)
    {
        poseDataBlob.setTargetTransformTS(tf);
    }

    Transform PoseCommunicator::getTargetFrameTransformTS()
    {
        return poseDataBlob.getTargetFrame().transform;
    }



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
        PoseDataSyncBlob pdBlob {poseDataBlob.getCopy()};
        if (pdBlob.getLocalTileFrame().transform==pdBlob.getStartLocalTileFrame().transform)
        {
            return false;
        }
        else
        {
            return true;
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
        return shouldflush;
        // bool retVal {false};
        // if (shouldflush)
        // {
        //     // shouldflush = false;
        //     return true;
        // }
        // else
        // {
        //     return shouldflush;
        // }
    }

    void PoseCommunicator::flushDone()
    {
        std::lock_guard<std::mutex> lock(mtx_flush);
        shouldflush = false;
    }

    std::vector<Walls> PoseCommunicator::requestWallStates()
    {
        mtx_wallStatesRequest.lock();
        wantsWallStates = true;
        mtx_wallStatesRequest.unlock();

        bool wait {true};
        while (wait)
        {
            mtx_wallStatesRequest.lock();
            wait = wantsWallStates;
            mtx_wallStatesRequest.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::lock_guard<std::mutex> lock(mtx_wallStatesReadWrite);
        return wallStates;

    }

    bool PoseCommunicator::checkWantsWallStates()
    {

        std::lock_guard<std::mutex> lock(mtx_wallStatesRequest);
        return wantsWallStates;

    }

    void PoseCommunicator::setWallStates(std::vector<Walls> states)
    {
        mtx_wallStatesReadWrite.lock();
        wallStates = states;

        mtx_wallStatesReadWrite.unlock();
        mtx_wallStatesRequest.lock();

        wantsWallStates = false;
        mtx_wallStatesRequest.unlock();
    }

}