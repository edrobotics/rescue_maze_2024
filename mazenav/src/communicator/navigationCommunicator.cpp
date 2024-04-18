#include "communicator/navigationCommunicator.h"

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

    void NavigationCommunicator::clearAllCommands()
    {
        mtx_commands.lock();

        while (!commands.empty())
        {
            commands.pop();
        }
        
        mtx_commands.unlock();
    }
} // namespace communication