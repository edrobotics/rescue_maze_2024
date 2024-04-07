#pragma once

#include <vector>

#include "communicator/communicator.h"
#include "globalNav/map/mazeMap.h"
#include "globalNav/map/mazePosition.h"

#define START_X LEVELSIZE/2
#define START_Y LEVELSIZE/2
#define STARTLEVEL 0

class MazeNavigator
{
    private:
    communication::Communicator* communicatorSingleton;
    MazeMap mazeMap = MazeMap();
    std::vector<MazePosition> knownUnexploredTilePositions;
    MazePosition currentPosition = MazePosition(START_X, START_Y, STARTLEVEL);

    bool followingAPath();

    public:
    MazeNavigator(communication::Communicator* communicatorInstance) : communicatorSingleton(communicatorInstance) {};
    void makeNavigationDecision();
    void exploreMaze();
    void followPath();
};