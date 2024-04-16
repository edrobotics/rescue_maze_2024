#include "localNav/MiniController.h"


void MiniController::start(communication::Communicator* gComm)
{
    runThread = std::thread{&MiniController::runLoopFunc, this, gComm};
}


void MiniController::waitForfinish()
{
    runThread.join();
}


void MiniController::runLoopFunc(communication::Communicator* gComm)
{
    globComm = gComm;
    char command {};
    while (true)
    {
        std::cin >> command;

        switch (command)
        {
            case 'w':
                globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
                break;
            case 'a':
                globComm->navigationComm.pushCommand(communication::DriveCommand::turnLeft);
                break;
            case 'd':
                globComm->navigationComm.pushCommand(communication::DriveCommand::turnRight);
                break;
            default:
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

        }
    }
}