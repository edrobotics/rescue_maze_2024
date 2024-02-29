#include "communicator/communicator.h"

namespace communication
{


    void Navigation::pushCommand(DriveCommand command)
    {
        commands_mutex.lock();
        commands.push(command);
        commands_mutex.unlock();
    }

    DriveCommand Navigation::getCommand(bool pop)
    {
        DriveCommand dC;
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
            dC = DriveCommand::noAction;
        }
        commands_mutex.unlock();

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