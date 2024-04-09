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

        // Lock mutex
        // const std::lock_guard<std::mutex> lock(mtx_general);

        worldFrame = pComm.worldFrame.getWithoutChildren();
        // std::cout << "Copied worldframe" << std::endl;
        localTileFrame = pComm.localTileFrame.getWithoutChildren();
        // std::cout << "Copied localTileFrame" << std::endl;
        localTileFrame.setParentTS(&worldFrame);
        // std::cout << "Set localTileFrame parent" << std::endl;
        robotFrame = pComm.robotFrame.getWithoutChildren();
        // std::cout << "Copied robotFrame" << std::endl;
        robotFrame.setParentTS(&localTileFrame);
        // std::cout << "Set robotFrame parent" << std::endl;
        
        robotSpeed = pComm.robotSpeed.getWithoutChildren();
        // std::cout << "Set robotSpeed" << std::endl;
        return *this;
    }

    PoseCommunicator::PoseCommunicator(const PoseCommunicator& pComm)
    {
        *this = pComm;
    }

}