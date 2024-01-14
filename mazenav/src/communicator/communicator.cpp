#include <communicator.h>

namespace communication
{
    void Navigation::pushCommand(driveCommand command)
    {
        commands_mutex.lock();
        commands.push(command);
        commands_mutex.unlock();
    }

    driveCommand Navigation::getCommand()
    {
        driveCommand dC;
        commands_mutex.lock();
        if (!commands.empty())
        {
            dC = commands.front();
        }
        else
        {
            dC = driveCommand::noAction;
        }
        commands_mutex.unlock();

        return dC;
    }
}