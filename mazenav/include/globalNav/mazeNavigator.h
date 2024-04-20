#pragma once

#include <vector>
#include <chrono>
#include <thread>
#include <stack>

#include "communicator/communicator.h"
#include "communicator/navigationCommunicator.h"
#include "fusion/Victim.h"
#include "globalNav/map/mazeMap.h"
#include "globalNav/map/position.h"
#include "globalNav/map/mazePath.h"
#include "globalNav/search/pathFinder.h"
#include "globalNav/map/directions.h"

#define START_X LEVELSIZE/2
#define START_Y LEVELSIZE/2
#define START_LEVEL 0
#define START_DIRECTION GlobalDirections::North

#define LOOP_SLEEPTIME std::chrono::milliseconds(20)
#define START_SLEEPTIME std::chrono::milliseconds(50)
#define SHORT_WAIT_SLEEPTIME std::chrono::milliseconds(4)
#define DRIVE_AND_TURN_TIME std::chrono::seconds(7)
#define END_BUFFER_TIME std::chrono::seconds(10)
#define END_SLEEP_TIME std::chrono::seconds(100)

class MazeNavigator
{
    private:
    communication::Communicator* communicatorSingleton;

    std::stack<MazePosition> knownUnexploredTilePositions;
    std::stack<MazePosition> checkpointedKnownUnexploredTilePositions;
    
    bool shouldReturnFromTile = false;
    bool sentNewDriveCommand = false;

    const MazePosition StartPosition{START_X, START_Y, START_LEVEL};
    MazePosition currentPosition = StartPosition;
    GlobalDirections currentDirection = START_DIRECTION;
    MazePosition latestCheckpointPosition = currentPosition;

    MazeMap mazeMap;
    PathFinder pathFinder{&mazeMap};
    MazePath pathToFollow;

    bool lackOfProgressActive = false;
    bool returningBecauseTime = false;

    bool endConditionsAreMet();
    void finishRun();

    void exploreMaze();

    bool anyTilesInPath();
    void followPath();
    void startFollowingPathToLastUnexploredTile();
    MazePath pathTo(MazePosition toPosition);

    void giveLowLevelInstruction(communication::DriveCommand command);

    void addExplorableNeighborsToExplorationStack();
    void addNeighborToExplorationStackIfExplorable(LocalDirections direction);
    bool canExploreNeighborInDirection(LocalDirections neighborDirection);
    void goToNeighborInDirection(LocalDirections direction);
    void turnToDirection(LocalDirections direction);
    void driveTile();
    MazePosition getNeighborInDirection(LocalDirections direction);

    void checkFlagsUntilDriveIsFinished();
    bool driveIsFinished();

    void handleActivePanicFlags();
    bool lackOfProgressFlagRaised();
    bool victimIsAvailable();
    bool droveHalfTileFlagRaised();
    void waitForFlag(communication::PanicFlags panicFlag);
    void handleVictimFlag();

    void updatePosition(const communication::TileDriveProperties& tileDriveProperties);
    void updatePositionFromDirection();
    void updateInfoFromRamp();
    void updateInfoFromNewRamp();
    void couldNotCreateNewRamp();
    void updateInfoFromOldRamp();

    void updateMap(const communication::TileDriveProperties& tileDriveProperties);
    std::vector<Tile::TileProperty> getWallProperties(std::vector<communication::Walls> walls);
    Tile::TileProperty wallToTileProperty(communication::Walls wall);
    Tile::TileProperty wallPropertyInDirection(LocalDirections direction);

    GlobalDirections localToGlobalDirection(LocalDirections localDirections);
    LocalDirections globalToLocalDirection(GlobalDirections globalDirections);

    void saveCheckpointInfo();
    void lackOfProgress();
    void lackOfProgressInactive();

    void returnIfLittleTime();
    std::chrono::seconds estimateTimeForPath(MazePath path);

    bool wallVectorHasWall(std::vector<communication::Walls> walls, communication::Walls searchWall);

    void logTileProperties(std::vector<Tile::TileProperty> properties);
    void logPosition();
    void logDirection();

    void logToFile(std::string message);
    void logToConsole(std::string message);
    void logToConsoleAndFile(std::string message);
    void logAndThrowException(std::string logText);

    public:
    MazeNavigator(communication::Communicator* communicatorInstance) : communicatorSingleton(communicatorInstance) {};
    void init();
    void makeNavigationDecision();
    void followLeftWall();
    void updateInfoAfterDriving();
};