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
            case 's':
                globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
                break;
            case 'a':
                globComm->navigationComm.pushCommand(communication::DriveCommand::turnLeft);
                break;
            case 'd':
                globComm->navigationComm.pushCommand(communication::DriveCommand::turnRight);
                break;
            case 'v':
                globComm->panicFlagComm.raiseFlag(communication::PanicFlags::victimDetected);
                break;
            case 'b':
                globComm->panicFlagComm.raiseFlag(communication::PanicFlags::sawBlackTile);
                break;
            case 'l':
                globComm->panicFlagComm.raiseFlag(communication::PanicFlags::lackOfProgressActivated);
                break;
            case 'r':
                globComm->panicFlagComm.raiseFlag(communication::PanicFlags::onRamp);
                break;
            default:
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;

        }
    }
}