#include "communicator/communicator.h"

namespace communication
{


    void NavigationCommunicator::pushCommand(DriveCommand command)
    {
        mtx_commands.lock();
        commands.push(command);
        mtx_commands.unlock();
    }

    DriveCommand NavigationCommunicator::popCommand()
    {
        DriveCommand queuedCommand;
        mtx_commands.lock();
        if (!commands.empty())
        {
            queuedCommand = commands.front();
            commands.pop();
        }
        else
        {
            queuedCommand = DriveCommand::noAction;
        }
        mtx_commands.unlock();

        return queuedCommand;
    }

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
        
        robotSpeed = pComm.robotSpeed.getWithoutChildren();

        freshness = pComm.freshness;

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

    CoordinateFrame PoseCommunicator::getTargetFrameTS()
    {
        std::lock_guard<std::mutex> lock(mtx_general);
        return targetFrame;
    }

    void PoseCommunicator::setTargetFrameTransformTS(Transform tf)
    {
        mtx_general.lock();
        targetFrame.applyTransform(tf);
        mtx_general.unlock();
    }

}