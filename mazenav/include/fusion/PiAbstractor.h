#pragma once

// Abstracts the Pi hardware to make it thread safe.

#include <mutex>

#ifdef ENV_PI
#include "wiringPi.h"
#endif


// Only one instance of this class may exist at any given time
class PiAbstractor
{
    private:

        std::mutex mtx_general {};

    public:
        enum class PinMode
        {
            inputPullup,
            inputPulldown,
            output,
        };

        // Call before doing anything
        void init();
        
        void pinModeTS(int pin, PinMode mode);
        void digitalWriteTS(int pin, bool state);
        bool digitalReadTS(int pin);

};