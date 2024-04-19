#include "communicator/logger.h"

using namespace communication;
using namespace std::chrono;

namespace communication
{
    void Logger::logToFile(std::string logMessage)
    {
        mtx_logging.lock();

        if (!logFile.is_open())
        {
            logFile.open(LogFileName, std::ios_base::app);
        }

        if (!logFile){
            std::cerr << "No file?" << std::endl;
            return;
        }

        logFile << getTimestampAsString() << ": " << logMessage << "\n";

        logFile.flush();
        logFile.close();
        
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

    std::string Logger::getTimestampAsString()
    {
        //Time now as a unix timestamp
        return std::to_string(duration_cast<seconds>(system_clock::now().time_since_epoch()).count());
    }

    std::string Logger::getLogFileNumber() const
    {
        int i;
        for (i = 0; std::filesystem::exists(LOGFILE_STARTSTRING + std::to_string(i) + LOGFILE_ENDSTRING); i++);
        return std::to_string(i);
    }
}