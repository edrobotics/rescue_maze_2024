#include "globalNav/mazeNavigator.h"

void MazeNavigator::makeNavigationDecision()
{
    if (anyTilesInPath()) 
        followPath();
    else
        exploreMaze();
}

bool MazeNavigator::anyTilesInPath()
{
    return !pathToFollow.isEmpty();
}

void MazeNavigator::followPath()
{
    auto globalDirectionOfNextPosition = mazeMap.neighborToDirection(currentPosition, pathToFollow.getNextPosition());
    if (globalDirectionOfNextPosition.has_value())
        goToNeighborInDirection(globalToLocalDirection(globalDirectionOfNextPosition.value()));
}

void MazeNavigator::exploreMaze()
{
    if (shouldTurnAroundAndGoBack)
    {
        goToNeighborInDirection(LocalDirections::Back);
    }
    else if (!exploreBestNeighbor())
    {
        startFollowingPathToLastUnexploredTile();
    }
}

bool MazeNavigator::exploreBestNeighbor()
{
    if (canExploreNeighborInDirection(LocalDirections::Left)) // Very sorry for really ugly code, I could not find a better way
        goToNeighborInDirection(LocalDirections::Left);
    else if (canExploreNeighborInDirection(LocalDirections::Front))
        goToNeighborInDirection(LocalDirections::Front);
    else if (canExploreNeighborInDirection(LocalDirections::Right))
        goToNeighborInDirection(LocalDirections::Right);
    else if (canExploreNeighborInDirection(LocalDirections::Back))
        goToNeighborInDirection(LocalDirections::Back);
    else return false;

    return true;
}

bool MazeNavigator::canExploreNeighborInDirection(LocalDirections neighborDirection)
{
    return mazeMap.neighborIsAvailableAndUnexplored(currentPosition, localToGlobalDirection(neighborDirection));
}

void MazeNavigator::startFollowingPathToLastUnexploredTile()
{
    pathToFollow = pathTo(knownUnexploredTilePositions.back());
    knownUnexploredTilePositions.pop_back();

    followPath();
}

MazePath MazeNavigator::pathTo(MazePosition toPosition)
{
    pathFinder.findPathTo(currentPosition, toPosition);
}

void MazeNavigator::goToNeighborInDirection(LocalDirections direction)
{
    turnToDirection(direction);
    driveTile();
}

void MazeNavigator::turnToDirection(LocalDirections direction)
{
    if (direction == LocalDirections::Right) giveLowLevelInstruction(communication::DriveCommand::turnRight);
    if (direction == LocalDirections::Left) giveLowLevelInstruction(communication::DriveCommand::turnLeft);
    if (direction == LocalDirections::Back) giveLowLevelInstruction(communication::DriveCommand::turnBack);
    currentDirection = localToGlobalDirection(direction);
}

void MazeNavigator::driveTile()
{
    giveLowLevelInstruction(communication::DriveCommand::driveForward);
}

void MazeNavigator::giveLowLevelInstruction(communication::DriveCommand command)
{
    communicatorSingleton->navigationComm.pushCommand(command);
}

GlobalDirections MazeNavigator::localToGlobalDirection(LocalDirections localDirections)
{
    int directionInt = (int)currentDirection + (int)localDirections; // This works because of the order in the enums
    directionInt = directionInt % DIRECTIONS_AMOUNT;
    return (GlobalDirections)directionInt;
}

LocalDirections MazeNavigator::globalToLocalDirection(GlobalDirections globalDirections)
{
    int directionInt = (int)globalDirections - (int)currentDirection; // This works because of the order in the enums
    if (directionInt < 0) directionInt += DIRECTIONS_AMOUNT;
    return (LocalDirections)directionInt;
}

void MazeNavigator::updateInfoFromOldRamp()
{
    currentPosition = mazeMap.positionAfterUsingRamp(currentPosition, currentDirection);
}

void MazeNavigator::updateInfoFromNewRamp()
{
    MazePosition previousPosition = currentPosition;
    currentPosition.levelIndex = 1;
    currentPosition.tileX = START_X;
    currentPosition.tileY = START_Y;

    mazeMap.createNewRamp(previousPosition, currentPosition, currentDirection);

    // #error think through
}