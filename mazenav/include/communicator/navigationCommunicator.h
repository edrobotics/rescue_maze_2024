#pragma once

#include <queue>
#include <mutex>

namespace communication
{
    // Possible drive commands that can be issued from global navigation to local navigation
    enum class DriveCommand
    {
        driveForward,
        driveBackward,
        turnLeft,
        turnRight,
        turnBack,
        noAction,
        init,
        driveCommand_num,
    };

    class NavigationCommunicator
    {
        public:
        void pushCommand(DriveCommand command);
        // Gets the next command from the command queue and removes it from the queue
        DriveCommand popCommand();
        //Should only be used for LOP
        void clearAllCommands();

        private:
        std::mutex mtx_commands {};
        std::queue<DriveCommand> commands {};
    };
} // namespace communication