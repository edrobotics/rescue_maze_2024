#include "communicator/PoseDataSyncBlob.h"

namespace communication
{
    PoseDataSyncBlob::PoseDataSyncBlob()
    {
        robotFrame.transform.pos_x = GRID_SIZE/2;
        robotFrame.transform.pos_y = GRID_SIZE/2;
        lastRobotFrame = robotFrame.getWithoutChildren();
    }

    void PoseDataSyncBlob::incrementHistoryIndex()
    {
        ++historyIndex;
        historyIndex = historyIndex % HISTORY_NUM;
    }

    void PoseDataSyncBlob::calcAverage(CoordinateFrame& result)
    {
        // Sort array to filter out most extreme values
        // std::array<CoordinateFrame, HISTORY_NUM> sorted {robotSpeeds};
        // std::sort(sorted.begin(), sorted.end());

        double xSum {0};
        double ySum {0};
        double zSum {0};

        // for (int i=1;i<HISTORY_NUM-1;++i)
        // {
        //     xSum+=sorted.at(i).transform.pos_x;
        //     ySum+=sorted.at(i).transform.pos_y;
        //     zSum+=sorted.at(i).transform.rot_z;
        // }
        // result.transform.pos_x = xSum/(HISTORY_NUM-2);
        // result.transform.pos_y = ySum/(HISTORY_NUM-2);
        // result.transform.rot_z = zSum/(HISTORY_NUM-2);

        for (int i=0;i<HISTORY_NUM;++i)
        {
            xSum += robotSpeeds.at(i).transform.pos_x;
            ySum += robotSpeeds.at(i).transform.pos_y;
            zSum += robotSpeeds.at(i).transform.rot_z;
        }
        result.transform.pos_x = xSum/static_cast<double>(HISTORY_NUM);
        result.transform.pos_y = ySum/static_cast<double>(HISTORY_NUM);
        result.transform.rot_z = zSum/static_cast<double>(HISTORY_NUM);

    }

    PoseDataSyncBlob PoseDataSyncBlob::borrow()
    {
        // Forbid write access
        mtx_writeRights.lock();
        return *this;
    }

    void PoseDataSyncBlob::giveBack(PoseDataSyncBlob blob)
    {
        // Forbid reads
        // The borrow function guarantees that only one object is ever borrowed, so if you never giveBack without borrowing first, you should be safe.
        mtx_readRights.lock();
        *this = blob;
        mtx_readRights.unlock();
        mtx_writeRights.unlock();
    }




    PoseDataSyncBlob& PoseDataSyncBlob::operator= (const PoseDataSyncBlob& pdBlob)
    {

        worldFrame = pdBlob.worldFrame.getWithoutChildren();
        
        localTileFrame = pdBlob.localTileFrame.getWithoutChildren();
        localTileFrame.setParentTS(&worldFrame);
        
        robotFrame = pdBlob.robotFrame.getWithoutChildren();
        robotFrame.setParentTS(&localTileFrame);

        lastRobotFrame = pdBlob.lastRobotFrame.getWithoutChildren();
        lastRobotFrame.setParentTS(&localTileFrame);
        // std::cout << "copying times... ";
        curRobotTime = pdBlob.curRobotTime;
        lastRobotTime = pdBlob.lastRobotTime;
        // std::cout << "done\n";

        robotSpeedAvg = pdBlob.robotSpeedAvg.getWithoutChildren();
        robotSpeedCur = pdBlob.robotSpeedCur.getWithoutChildren();
        // std::cout << "copying robotSpeeds... ";
        robotSpeeds = pdBlob.robotSpeeds;
        // std::cout << "done\n";
        historyIndex = pdBlob.historyIndex;

        targetFrame = pdBlob.targetFrame.getWithoutChildren();
        targetFrame.setParentTS(&localTileFrame);

        startLocalTileFrame = pdBlob.startLocalTileFrame.getWithoutChildren();

        return *this;
    }

    PoseDataSyncBlob::PoseDataSyncBlob(const PoseDataSyncBlob& pdBlob)
    {
        *this = pdBlob;
    }



    CoordinateFrame PoseDataSyncBlob::getWorldFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return worldFrame.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getLocalTileFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return localTileFrame.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getRobotFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return robotFrame.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getLastRobotFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return lastRobotFrame.getWithoutChildren();
    }

    std::chrono::steady_clock::time_point PoseDataSyncBlob::getCurRobotTime()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return curRobotTime;
    }

    std::chrono::steady_clock::time_point PoseDataSyncBlob::getLastRobotTime()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return lastRobotTime;
    }

    CoordinateFrame PoseDataSyncBlob::getRobotSpeedAvg()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return robotSpeedAvg.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getRobotSpeedCur()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return robotSpeedCur.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getTargetFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return targetFrame.getWithoutChildren();
    }

    CoordinateFrame PoseDataSyncBlob::getStartLocalTileFrame()
    {
        std::lock_guard<std::mutex> lock(mtx_readRights);
        return startLocalTileFrame.getWithoutChildren();
    }

}