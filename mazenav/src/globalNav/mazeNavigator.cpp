#include "globalNav/mazeNavigator.h"

void MazeNavigator::init()
{
    std::this_thread::sleep_for(START_SLEEPTIME);
    logToConsoleAndFile("sending init");
    giveLowLevelInstruction(communication::DriveCommand::init);

    checkFlagsUntilDriveIsFinished();

    communication::TileDriveProperties tileDriveProperties = communicatorSingleton->tileInfoComm.readLatestTileProperties();
    updateMap(tileDriveProperties);
}

void MazeNavigator::makeNavigationDecision()
{
    addExplorableNeighborsToExplorationStack();

    if (anyTilesInPath()) 
        followPath();
    else
        exploreMaze();
}

void MazeNavigator::followLeftWall()
{
    //sendResetCommand
    std::this_thread::sleep_for(DRIVE_AND_TURN_TIME);
    giveLowLevelInstruction(communication::DriveCommand::init);

    while (!driveIsFinished()) std::this_thread::sleep_for(LOOP_SLEEPTIME);
    communication::TileDriveProperties tileDriveProperties = communicatorSingleton->tileInfoComm.readLatestTileProperties();
    auto walls = tileDriveProperties.wallsOnNewTile;
    
    bool frontWall = wallVectorHasWall(walls, communication::Walls::FrontWall);
    bool leftWall = wallVectorHasWall(walls, communication::Walls::LeftWall);
    bool rightWall = wallVectorHasWall(walls, communication::Walls::RightWall);

    if (!tileDriveProperties.droveTile && tileDriveProperties.tileColourOnNewTile == TileColours::Black)
        frontWall = true;

    if (!leftWall)
        turnToDirection(LocalDirections::Left);
    else if (frontWall)
    {
        if (rightWall) turnToDirection(LocalDirections::Back);
        else turnToDirection(LocalDirections::Right);
    }
    
    driveTile();
}

bool MazeNavigator::wallVectorHasWall(std::vector<communication::Walls> walls, communication::Walls searchWall)
{
    for (communication::Walls& vecWall : walls)
    {
        if (vecWall == searchWall)
            return true;
    }
    return false;
}

bool MazeNavigator::anyTilesInPath()
{
    return !pathToFollow.isEmpty();
}

void MazeNavigator::followPath()
{
    MazePosition nextPositionInPath = pathToFollow.getNextPosition();
    while (nextPositionInPath == currentPosition)
        pathToFollow.getNextPosition();
    
    auto globalDirectionOfNextPosition = mazeMap.neighborToDirection(currentPosition, pathToFollow.getNextPosition());
    if (globalDirectionOfNextPosition.has_value())
        goToNeighborInDirection(globalToLocalDirection(globalDirectionOfNextPosition.value()));
    else
    {
        logToConsoleAndFile("NEXT TILE IS NOT NEIGHBOR");
        throw std::exception();
    }
}

void MazeNavigator::exploreMaze()
{
    if (shouldReturnFromTile)
    {
        goToNeighborInDirection(LocalDirections::Back);
        shouldReturnFromTile = false;
    }
    else 
    {
        startFollowingPathToLastUnexploredTile();
    }
}

void MazeNavigator::addExplorableNeighborsToExplorationStack()
{
    addNeighborToExplorationStackIfExplorable(LocalDirections::Back); //Reverse exploration priority
    addNeighborToExplorationStackIfExplorable(LocalDirections::Right);
    addNeighborToExplorationStackIfExplorable(LocalDirections::Front);
    addNeighborToExplorationStackIfExplorable(LocalDirections::Left);
}

void MazeNavigator::addNeighborToExplorationStackIfExplorable(LocalDirections direction)
{
    if (canExploreNeighborInDirection(direction))
    {
        MazePosition tileToAdd = getNeighborInDirection(direction);
        knownUnexploredTilePositions.push(tileToAdd);
        logToConsoleAndFile("Added tile " + tileToAdd.toLoggable() + " to explorationStack");
    }
}

bool MazeNavigator::canExploreNeighborInDirection(LocalDirections neighborDirection)
{
    return mazeMap.neighborIsAvailableAndUnexplored(currentPosition, localToGlobalDirection(neighborDirection));
}

MazePosition MazeNavigator::getNeighborInDirection(LocalDirections direction)
{
    return mazeMap.neighborInDirection(currentPosition, localToGlobalDirection(direction));
}

void MazeNavigator::startFollowingPathToLastUnexploredTile()
{
    if (!knownUnexploredTilePositions.empty()) 
    {
        while (mazeMap.tileHasProperty(knownUnexploredTilePositions.top(), Tile::TileProperty::Explored) || knownUnexploredTilePositions.top() == currentPosition)
            knownUnexploredTilePositions.pop();
        
        pathToFollow = pathTo(knownUnexploredTilePositions.top());
        knownUnexploredTilePositions.pop();
    }
    else
    {
        pathToFollow = pathTo(MazePosition(START_X, START_Y, START_LEVEL));
    }

    followPath();
}

MazePath MazeNavigator::pathTo(MazePosition toPosition)
{
    logToConsoleAndFile("Finding path to" + std::to_string(toPosition.tileX) + "," + std::to_string(toPosition.tileY));
    MazePath path = pathFinder.findPathTo(currentPosition, toPosition);
    logToConsoleAndFile("Found " + path.toLoggable());
    return path;
}

void MazeNavigator::goToNeighborInDirection(LocalDirections direction)
{
    turnToDirection(direction);
    driveTile();
}

void MazeNavigator::turnToDirection(LocalDirections direction)
{
    if (direction == LocalDirections::Right) 
    {
        giveLowLevelInstruction(communication::DriveCommand::turnRight);
        logToConsoleAndFile("turning right");
    }
    if (direction == LocalDirections::Left)
    {
        giveLowLevelInstruction(communication::DriveCommand::turnLeft);
        logToConsoleAndFile("turning left");
    }
    if (direction == LocalDirections::Back)
    { 
        giveLowLevelInstruction(communication::DriveCommand::turnBack);
        logToConsoleAndFile("turning to back");
    }
    currentDirection = localToGlobalDirection(direction);
}

void MazeNavigator::driveTile()
{
    logToConsoleAndFile("Sending command to drive forward");
    giveLowLevelInstruction(communication::DriveCommand::driveForward);
    sentNewDriveCommand = true;
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

//***************************************************************************************************************************************
//************************************************************ INFO UPDATING ************************************************************
//***************************************************************************************************************************************

void MazeNavigator::updateInfoAfterDriving()
{
    if (!sentNewDriveCommand) return;
    sentNewDriveCommand = false;

    checkFlagsUntilDriveIsFinished();

    communication::TileDriveProperties tileDriveProperties = communicatorSingleton->tileInfoComm.readLatestTileProperties();
    updatePosition(tileDriveProperties);
    updateMap(tileDriveProperties);
}

void MazeNavigator::checkFlagsUntilDriveIsFinished()
{
    while (!driveIsFinished())
    {
        handleActivePanicFlags();
        std::this_thread::sleep_for(LOOP_SLEEPTIME);
    }
}

bool MazeNavigator::driveIsFinished()
{
    return communicatorSingleton->tileInfoComm.hasNewTileInfo();
}

void MazeNavigator::handleActivePanicFlags()
{
    if (victimFlagRaised())
    {
        std::vector<Victim> victims = communicatorSingleton->victimDataComm.getAllNonStatusVictims();
        for (auto i = victims.begin(); i != victims.end(); i++)
        {
            if (!mazeMap.tileHasProperty(currentPosition, Tile::TileProperty::HasVictim))
            {
                mazeMap.setTileProperty(currentPosition, Tile::TileProperty::HasVictim, true);
                communicatorSingleton->victimDataComm.addVictimToRescueQueue(*i);
            }
        }
    }
    if (lackOfProgressFlagRaised())
    {
        resetToLastCheckpoint();
    }
}

bool MazeNavigator::lackOfProgressFlagRaised()
{
    return communicatorSingleton->panicFlagComm.readFlagFromThread(communication::PanicFlags::lackOfProgressActivated, communication::ReadThread::globalNav);
}

bool MazeNavigator::victimFlagRaised()
{
    return communicatorSingleton->panicFlagComm.readFlagFromThread(communication::PanicFlags::victimDetected, communication::ReadThread::globalNav);
}

void MazeNavigator::updatePosition(const communication::TileDriveProperties& tileDriveProperties)
{
    if (!tileDriveProperties.droveTile)
    {
        if (tileDriveProperties.tileColourOnNewTile == TileColours::Black)
            mazeMap.setTileProperty(getNeighborInDirection(globalToLocalDirection(currentDirection)), Tile::TileProperty::Black, true);
    }
    else if(tileDriveProperties.usedRamp)
    {
        updateInfoFromRamp();
    }
    else
    {
        updatePositionNormally();
    }
}

void MazeNavigator::updatePositionNormally()
{
    currentPosition = mazeMap.neighborInDirection(currentPosition, currentDirection);
}

void MazeNavigator::updateInfoFromRamp()
{
    if(mazeMap.rampHasBeenUsedBefore(currentPosition, currentDirection))
    {
        updateInfoFromOldRamp();
    }
    else
    {
        if (mazeMap.canCreateNewRamps())
            updateInfoFromNewRamp();
        else
            couldNotCreateNewRamp();
    }
}

void MazeNavigator::couldNotCreateNewRamp() //DELETE AFTER LINKÖPING
{
    shouldReturnFromTile = true; // Since we cannot create a new ramp we need to return via the ramp
    updatePositionNormally();
    mazeMap.setTileProperty(currentPosition, Tile::TileProperty::Black, true);
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

    mazeMap.createNewRampAndLevel(previousPosition, currentPosition, currentDirection);
}

void MazeNavigator::updateMap(const communication::TileDriveProperties& tileDriveProperties)
{
    logDirection();

    std::vector<Tile::TileProperty> tileProperties = getWallProperties(tileDriveProperties.wallsOnNewTile);

    if (tileDriveProperties.tileColourOnNewTile == TileColours::Checkpoint){
        saveCheckpointInfo();
        tileProperties.push_back(Tile::TileProperty::Checkpoint);
    }
    else if (tileDriveProperties.tileColourOnNewTile == TileColours::Blue)
        tileProperties.push_back(Tile::TileProperty::Blue);

    logTileProperties(tileProperties);
    mazeMap.makeTileExploredWithProperties(currentPosition, tileProperties);
}

std::vector<Tile::TileProperty> MazeNavigator::getWallProperties(std::vector<communication::Walls> walls)
{
    std::vector<Tile::TileProperty> tileProperties;

    for (std::size_t i = 0; i < walls.size(); i++)
    {
        tileProperties.push_back(wallToTileProperty(walls[i]));
    }
    
    return tileProperties;
}

Tile::TileProperty MazeNavigator::wallToTileProperty(communication::Walls wall)
{
    if (wall == communication::Walls::FrontWall) return wallPropertyInDirection(LocalDirections::Front);
    if (wall == communication::Walls::LeftWall) return wallPropertyInDirection(LocalDirections::Left);
    if (wall == communication::Walls::BackWall) return wallPropertyInDirection(LocalDirections::Back);
    return wallPropertyInDirection(LocalDirections::Right);
}

Tile::TileProperty MazeNavigator::wallPropertyInDirection(LocalDirections direction)
{
    GlobalDirections globalDirection = localToGlobalDirection(direction);
    if (globalDirection == GlobalDirections::North) return Tile::TileProperty::WallNorth;
    if (globalDirection == GlobalDirections::West) return Tile::TileProperty::WallWest;
    if (globalDirection == GlobalDirections::South) return Tile::TileProperty::WallSouth;
    return Tile::TileProperty::WallEast;
}

void MazeNavigator::resetToLastCheckpoint()
{
    mazeMap.resetSinceLastCheckpoint();
    currentPosition = latestCheckpointPosition;
    knownUnexploredTilePositions = checkpointedKnownUnexploredTilePositions;
}

void MazeNavigator::saveCheckpointInfo()
{
    latestCheckpointPosition = currentPosition;
    checkpointedKnownUnexploredTilePositions = knownUnexploredTilePositions;
    mazeMap.checkpointData();
}

void MazeNavigator::returnIfLittleTime()
{
    MazePath pathHome = pathTo(MazePosition(START_X, START_Y, START_LEVEL));
    if (communicatorSingleton->timer.timeRemaining() <= estimateTimeForPath(pathHome) + END_WAIT_TIME)
    {
        logToConsoleAndFile("RETURNING HOME");
        pathToFollow = pathHome;
    }
}

std::chrono::seconds MazeNavigator::estimateTimeForPath(MazePath path)
{
    return path.getPositionAmount() * DRIVE_AND_TURN_TIME;
}

void MazeNavigator::logDirection()
{
    std::string logString = "Current direction: ";

    if (currentDirection == GlobalDirections::North) logString += "North";
    if (currentDirection == GlobalDirections::South) logString += "South";
    if (currentDirection == GlobalDirections::West) logString += "West";
    if (currentDirection == GlobalDirections::East) logString += "East";

    logToConsoleAndFile(logString);
}

void MazeNavigator::logTileProperties(std::vector<Tile::TileProperty> properties)
{
    std::string logString = "TileProperties: ";
    for (Tile::TileProperty& property : properties)
    {
        if (property == Tile::TileProperty::WallNorth) logString += ",nW";
        if (property == Tile::TileProperty::WallWest) logString += ",wW";
        if (property == Tile::TileProperty::WallSouth) logString += ",sW";
        if (property == Tile::TileProperty::WallEast) logString += ",eW";
        if (property == Tile::TileProperty::Black) logString += ",Bk";
        if (property == Tile::TileProperty::Checkpoint) logString += ",Ch";
    }
    logToConsoleAndFile(logString);
}

void MazeNavigator::logToFile(std::string message)
{
    communicatorSingleton->logger.logToFile("globalNav: " + message);
}

void MazeNavigator::logToConsole(std::string message)
{
    communicatorSingleton->logger.logToConsole("globalNav: " + message);
}

void MazeNavigator::logToConsoleAndFile(std::string message)
{
    communicatorSingleton->logger.logToAll("globalNav: " + message);
}
