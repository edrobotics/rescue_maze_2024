#include "communicator/communicator.h"

namespace communication
{
    void MotorControllerCommunicator::setSpeeds(MotorControllers::MotorSpeeds speeds)
    {
        mtx_speeds.lock();
        this->speeds = speeds;
        mtx_speeds.unlock();
    }

    MotorControllers::MotorSpeeds MotorControllerCommunicator::getSpeeds()
    {
        mtx_speeds.lock();
        MotorControllers::MotorSpeeds spd {speeds};
        mtx_speeds.unlock();
        return spd;
    }


    PoseCommunicator::PoseCommunicator()
    {
        // robotSpeeds.fill(CoordinateFrame{nullptr});
        robotFrame.transform.pos_x = GRID_SIZE/2;
        robotFrame.transform.pos_y = GRID_SIZE/2;
        lastRobotFrame = robotFrame.getWithoutChildren();
    }

    PoseCommunicator& PoseCommunicator::operator=(const PoseCommunicator& pComm)
    {
        // std::cout << "Began assignment of PoseCommunicator" << std::endl;

        #warning should check if mutex should be locked here. Probably not, because of new usage of the mutex elsewhere.
        // Lock mutex
        // const std::lock_guard<std::mutex> lock(mtx_general);

        worldFrame = pComm.worldFrame.getWithoutChildren();
        
        localTileFrame = pComm.localTileFrame.getWithoutChildren();
        localTileFrame.setParentTS(&worldFrame);
        
        robotFrame = pComm.robotFrame.getWithoutChildren();
        robotFrame.setParentTS(&localTileFrame);

        lastRobotFrame = pComm.lastRobotFrame.getWithoutChildren();
        lastRobotFrame.setParentTS(&localTileFrame);

        targetFrame = pComm.targetFrame.getWithoutChildren();
        targetFrame.setParentTS(&localTileFrame);
        
        robotSpeedAvg = pComm.robotSpeedAvg.getWithoutChildren();
        // std::cout << "copying robotSpeeds... ";
        robotSpeeds = pComm.robotSpeeds;
        // std::cout << "done\n";
        historyIndex = pComm.historyIndex;

        freshness = pComm.freshness;
        updated = pComm.updated;

        // std::cout << "copying times... ";
        curRobotTime = pComm.curRobotTime;
        lastRobotTime = pComm.lastRobotTime;
        // std::cout << "done\n";

        return *this;
    }

    PoseCommunicator::PoseCommunicator(const PoseCommunicator& pComm)
    {
        *this = pComm;
    }

    void PoseCommunicator::setTargetFrameTS(CoordinateFrame& frame)
    {
        mtx_general.lock();
        targetFrame = frame;
        mtx_general.unlock();
    }

    CoordinateFrame PoseCommunicator::getTargetFrame()
    {
        // std::lock_guard<std::mutex> lock(mtx_general);
        return targetFrame;
    }

    void PoseCommunicator::setTargetFrameTransformTS(Transform tf)
    {
        mtx_general.lock();
        targetFrame.applyTransform(tf);
        mtx_general.unlock();
    }


    void PoseCommunicator::incrementHistoryIndex()
    {
        ++historyIndex;
        historyIndex = historyIndex % HISTORY_NUM;
    }


    void PoseCommunicator::calcRobotSpeedAvg(CoordinateFrame newSpeed)
    {
        // std::cout << "Begin speedCalc... ";
        robotSpeedCur = newSpeed;
        robotSpeeds.at(historyIndex) = robotSpeedCur;
        incrementHistoryIndex();
        calcAverage(robotSpeedAvg);
        // robotSpeedAvg = robotSpeedCur;
        // std::cout << "calc done" << "\n";
    }

    void PoseCommunicator::calcAverage(CoordinateFrame& result)
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

}