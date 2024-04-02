#pragma once

#include "communicator/communicator.h"

class MazeNavigator
{
    private:
    communication::Communicator* communicatorSingleton;
    public:
    MazeNavigator(communication::Communicator* communicatorInstance) : communicatorSingleton(communicatorInstance) {};
    void makeDecision();
};