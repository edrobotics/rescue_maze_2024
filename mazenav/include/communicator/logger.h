#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <chrono>

namespace communication
{

class Logger
{
    private:
    std::string getTimestampAsString();
    std::mutex mtx_logging;
    const std::string logFileName = (std::string)"/home/RCJ24LOG" + getTimestampAsString() + ".log";

    public:
    void logToFile(std::string logMessage);
    void logToConsole(std::string logMessage);
    void logToAll(std::string logMessage);
};

} //namespace communication