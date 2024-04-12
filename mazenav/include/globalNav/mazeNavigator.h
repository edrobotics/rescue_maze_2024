#pragma once

#include <vector>

#include "communicator/communicator.h"
#include "globalNav/map/mazeMap.h"
#include "globalNav/map/position.h"
#include "globalNav/map/mazePath.h"
#include "globalNav/search/pathFinder.h"
#include "globalNav/map/directions.h"

#define START_X LEVELSIZE/2
#define START_Y LEVELSIZE/2
#define START_LEVEL 0
#define START_DIRECTION GlobalDirections::North

class MazeNavigator
{
    private:
    communication::Communicator* communicatorSingleton;
    std::vector<MazePosition> knownUnexploredTilePositions;
    
    bool shouldTurnAroundAndGoBack = false;

    MazePosition currentPosition = MazePosition(START_X, START_Y, START_LEVEL);
    GlobalDirections currentDirection = START_DIRECTION;

    MazeMap mazeMap;

    PathFinder pathFinder{&mazeMap};
    MazePath pathToFollow;

    void exploreMaze();

    bool anyTilesInPath();
    void followPath();
    void startFollowingPathToLastUnexploredTile();
    MazePath pathTo(MazePosition toPosition);

    void giveLowLevelInstruction(communication::DriveCommand command);

    bool exploreBestNeighbor();
    bool canExploreNeighborInDirection(LocalDirections neighborDirection);
    void goToNeighborInDirection(LocalDirections direction);
    void turnToDirection(LocalDirections direction);
    void driveTile();

    void updateInfoFromNewRamp();
    void updateInfoFromOldRamp();

    GlobalDirections localToGlobalDirection(LocalDirections localDirections);
    LocalDirections globalToLocalDirection(GlobalDirections globalDirections);

    public:
    MazeNavigator(communication::Communicator* communicatorInstance) : communicatorSingleton(communicatorInstance) {};
    void makeNavigationDecision();
};