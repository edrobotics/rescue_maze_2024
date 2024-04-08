#include "communicator/logger.h"

using namespace communication;
using namespace std::chrono;

namespace communication
{
    void Logger::logToFile(std::string logMessage)
    {
        mtx_logging.lock();
        std::ofstream oFile(logFileName);

        if (!oFile){
            std::cerr << "No file?" << std::endl;
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

    std::string Logger::getTimestampAsString()
    {
        //Now unix timestamp
        return std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
    }
}