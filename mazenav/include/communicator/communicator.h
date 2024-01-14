#include <queue>
#include <mutex>

namespace communication
{
    enum driveCommand
    {
        driveForward,
        turnLeft,
        turnRight,
        turnBack,
        noAction
    };

    class Communicator
    {
        public:
        Navigation navigationComm;
    };

    class Navigation //Rename? - in general also
    {
        public:
        void pushCommand(driveCommand command);
        driveCommand getCommand();

        private:
        std::mutex commands_mutex;
        std::queue<driveCommand> commands;
    };
}