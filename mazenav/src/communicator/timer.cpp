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
        return duration_cast<seconds>(system_clock::now()-startTime).count() >= END_TIME_SECONDS;
    }

    std::chrono::seconds Timer::timeRemaining()
    {
        return duration_cast<seconds>(endTime-system_clock::now());
    }
}