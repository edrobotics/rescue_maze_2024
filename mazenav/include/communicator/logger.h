#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <chrono>
#include <filesystem>

namespace communication
{

class Logger
{
    private:
    #define LOGFILE_STARTSTRING (std::string)"/home/RCJ24LOG"
    #define LOGFILE_ENDSTRING (std::string) ".log"

    std::string getTimestampAsString();
    std::string getLogFileNumber() const;
    std::mutex mtx_logging;
    std::string logFileName;

    public:
    Logger::Logger() { logFileName = LOGFILE_STARTSTRING + getLogFileNumber() + LOGFILE_ENDSTRING; }
    void logToFile(std::string logMessage);
    void logToConsole(std::string logMessage);
    void logToAll(std::string logMessage);
};

} //namespace communication