#pragma once

#include <vector>

#include "communicator/communicator.h"
#include "globalNav/map/mazeMap.h"
#include "globalNav/map/mazePosition.h"
#include "globalNav/map/mazePath.h"
#include "globalNav/search/pathFinder.h"
#include "globalNav/map/directions.h"

#define START_X LEVELSIZE/2
#define START_Y LEVELSIZE/2
#define START_LEVEL 0
#define START_DIRECTION Directions::North

class MazeNavigator
{
    private:
    communication::Communicator* communicatorSingleton;
    std::vector<MazePosition> knownUnexploredTilePositions;

    MazePosition currentPosition = MazePosition(START_X, START_Y, START_LEVEL);
    Directions currentDirection = START_DIRECTION;

    MazeMap mazeMap = MazeMap(currentPosition);

    PathFinder pathFinder;
    MazePath pathToFollow;

    void exploreMaze();
    bool tilesInPath();
    void followPath();
    void startFollowingPathToLastUnexploredTile();
    void exploreBestNeighbor();
    MazePath pathTo(MazePosition toPosition);

    public:
    MazeNavigator(communication::Communicator* communicatorInstance) : communicatorSingleton(communicatorInstance) {};
    void makeNavigationDecision();
};