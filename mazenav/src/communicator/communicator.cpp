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

    void Logger::logToFile(std::string logMessage)
    {
        mtx_logging.lock();
        #warning What path??
        std::ofstream oFile("/home/RCJ24LOG.log");

        if (!oFile){
            std::cerr << "What error, no file?" << std::endl;
            return;
        }

        oFile << logMessage << "\n";

        oFile.flush();
        oFile.close();
        
        mtx_logging.unlock();
    }
    void Logger::logToConsole(std::string logMessage)
    {
        mtx_logging.lock();
        std::cout << logMessage << std::endl;
        mtx_logging.unlock();
    }
    void Logger::logToAll(std::string logMessage)
    {
        logToFile(logMessage);
        logToConsole(logMessage);
    }
}