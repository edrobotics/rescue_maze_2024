#pragma once

#include <chrono>
#include <mutex>

namespace communication
{
    #define END_TIME_MINUTES 8
    #define END_TIME_SECONDS 60 * END_TIME_MINUTES

    class Timer
    {
        private:
        std::mutex mtx_timer;
        std::chrono::_V2::system_clock::time_point startTime;
        std::chrono::_V2::system_clock::time_point endTime;

        public:
        bool timeIsOut();
        std::chrono::seconds timeRemaining();
        void startTimer();
    };
}