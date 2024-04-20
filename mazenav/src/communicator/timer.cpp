#include "communicator/timer.h"

using namespace std::chrono;

namespace communication
{
    void Timer::startTimer()
    {
        mtx_timer.lock();
        if (!timerStarted)
        {
            timerStarted = true;
            startTime = system_clock::now();
            endTime = startTime + seconds(END_TIME_SECONDS);
        }
        mtx_timer.unlock();
    }

    bool Timer::timeIsOut()
    {
        if (!timerStarted) return false;
        return duration_cast<seconds>(system_clock::now()-startTime).count() >= END_TIME_SECONDS;
    }

    std::chrono::seconds Timer::timeRemaining()
    {
        if (!timerStarted) return std::chrono::seconds(END_TIME_SECONDS);
        return duration_cast<seconds>(endTime-system_clock::now());
    }
}