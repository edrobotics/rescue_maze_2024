#include "communicator/communicator.h"

namespace communication
{

    void PoseCommunicator::setPose(RobotPose pose)
    {
        mtx_pose.lock();
        this->pose = pose;
        mtx_pose.unlock();
    }

    RobotPose PoseCommunicator::getPose()
    {
        RobotPose returnData {};
        mtx_pose.lock();
        returnData = this->pose;
        mtx_pose.unlock();
        return returnData;
    }

    void Navigation::pushCommand(driveCommand command)
    {
        commands_mutex.lock();
        commands.push(command);
        commands_mutex.unlock();
    }

    driveCommand Navigation::getCommand(bool pop)
    {
        driveCommand dC;
        commands_mutex.lock();
        if (!commands.empty())
        {
            dC = commands.front();
            if (pop)
            {
                commands.pop();
            }
        }
        else
        {
            dC = driveCommand::noAction;
        }
        commands_mutex.unlock();

        return dC;
    }
}