#pragma once

#include <thread>
#include <iostream>
#include <chrono>

#include "communicator/communicator.h"

class MiniController
{
    public:

        void start(communication::Communicator* gComm);


        void waitForfinish();
        void runLoopFunc(communication::Communicator* gComm);

    private:

        communication::Communicator* globComm {};

        std::thread runThread;

};