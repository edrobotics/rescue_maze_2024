#include "communicator/communicator.h"

namespace communication
{


    void NavigationCommunicator::pushCommand(DriveCommand command)
    {
        mtx_commands.lock();
        commands.push(command);
        mtx_commands.unlock();
    }

    DriveCommand NavigationCommunicator::getCommand(bool pop)
    {
        DriveCommand dC;
        mtx_commands.lock();
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
            dC = DriveCommand::noAction;
        }
        mtx_commands.unlock();

        return dC;
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


}