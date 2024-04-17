#include "globalNav/map/mazeMap.h"

MazeMap::MazeMap()
{ 
    mazeLevels.push_back(MazeLevel());
};

bool MazeMap::availableNeighborTilesAreExplored(MazePosition position)
{
    bool northAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, GlobalDirections::North);
    bool westAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, GlobalDirections::West);
    bool southAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, GlobalDirections::South);
    bool eastAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, GlobalDirections::East);
    return !northAvailableAndUnexplored && !westAvailableAndUnexplored && !southAvailableAndUnexplored && !eastAvailableAndUnexplored;
}

bool MazeMap::neighborIsAvailableAndUnexplored(MazePosition position, GlobalDirections neighborDirection)
{
    bool available = neighborIsAvailable(position, neighborDirection);
    bool explored = neighborHasProperty(position, neighborDirection, Tile::TileProperty::Explored);

    return available && !explored;
}

bool MazeMap::neighborIsAvailable(MazePosition position, GlobalDirections neighborDirection)
{
    return tileHasWallInDirection(position, neighborDirection) && 
          !tileHasProperty(neighborInDirection(position, neighborDirection), Tile::TileProperty::Black);
}

bool MazeMap::tileHasWallInDirection(MazePosition position, GlobalDirections wallDirection)
{
    return tileHasProperty(position, directionToWallProperty(wallDirection));
}

bool MazeMap::tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty)
{
    return mazeLevels[tilePosition.levelIndex].tileHasProperty(tilePosition, tileProperty);
}

bool MazeMap::neighborHasProperty(MazePosition basePosition, GlobalDirections neighborDirection, Tile::TileProperty tileProperty)
{
    MazePosition neighbor = neighborInDirection(basePosition, neighborDirection);
    return mazeLevels[basePosition.levelIndex].tileHasProperty(neighbor, tileProperty);
}

void MazeMap::setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState)
{
    mazeLevels[tilePosition.levelIndex].setTilePropertyAt(tilePosition, tileProperty, toState);
}

void MazeMap::makeTileExploredWithProperties(MazePosition tilePosition, std::vector<Tile::TileProperty> tileProperties)
{
    setTileProperty(tilePosition, Tile::TileProperty::Explored, true);
    for (auto i = tileProperties.begin(); i != tileProperties.end(); i++)
    {
        setTileProperty(tilePosition, *i, true);
    }
    uncheckpointedTiles.push_back(tilePosition);
}

Tile::TileProperty MazeMap::directionToWallProperty(GlobalDirections direction)
{
    switch (direction) // Apologizing for switch, but I could not find a better way
    {
    case GlobalDirections::North:
        return Tile::TileProperty::WallNorth;
    case GlobalDirections::West:
        return Tile::TileProperty::WallWest;
    case GlobalDirections::South:
        return Tile::TileProperty::WallSouth;
    default:
        return Tile::TileProperty::WallEast;
    }
}

MazePosition MazeMap::neighborInDirection(MazePosition basePosition, GlobalDirections neighborDirection)
{
    int xOffset = 0, yOffset = 0;
    switch (neighborDirection) // Apologizing for switch, but I could not find a better way
    {
    case GlobalDirections::North: 
        yOffset = -1;
        break;
    case GlobalDirections::West: 
        xOffset = -1;
        break;
    case GlobalDirections::South: 
        yOffset = 1;
        break;
    case GlobalDirections::East: 
        xOffset = 1;
        break;
    default:
        break;
    }

    return MazePosition(basePosition.tileX + xOffset, basePosition.tileY + yOffset, basePosition.levelIndex);
}

std::optional<GlobalDirections> MazeMap::neighborToDirection(MazePosition basePosition, MazePosition neighborPosition)
{
    if (basePosition.levelIndex != neighborPosition.levelIndex) return std::nullopt;

    if (neighborIsInDirection(basePosition, neighborPosition, GlobalDirections::North)) // Sorry for ugly code, I could not find a better way
        return GlobalDirections::North;
    if (neighborIsInDirection(basePosition, neighborPosition, GlobalDirections::West)) 
        return GlobalDirections::West;
    if (neighborIsInDirection(basePosition, neighborPosition, GlobalDirections::South)) 
        return GlobalDirections::South;
    if (neighborIsInDirection(basePosition, neighborPosition, GlobalDirections::East)) 
        return GlobalDirections::East;

    return std::nullopt;
}

bool MazeMap::neighborIsInDirection(MazePosition basePosition, MazePosition neighborPosition, GlobalDirections direction)
{
    return neighborInDirection(basePosition, direction) == neighborPosition;
}

void MazeMap::createNewRampAndLevel(MazePosition previousPosition, MazePosition newPosition, GlobalDirections direction)
{
    ramps.push_back(Ramp(previousPosition, newPosition, direction));
    createNewLevel();
    
    setTileProperty(previousPosition, Tile::TileProperty::ContainsRamp, true);
    setTileProperty(newPosition, Tile::TileProperty::ContainsRamp, true);
}

void MazeMap::createNewLevel()
{
    mazeLevels.push_back(MazeLevel());
}

bool MazeMap::canCreateNewRamps()
{
    if (ramps.size() == 0) return true;
    return false;
}

bool MazeMap::rampHasBeenUsedBefore(MazePosition rampPosition, GlobalDirections rampDirection)
{
    for (auto i = ramps.begin(); i != ramps.end(); i++)
    {
        bool isFirstPosition = i->getPositionInFirstLevel() == rampPosition && 
                               i->getDirectionInFirstLevel() == rampDirection;
        bool isSecondPosition = i->getPositionInSecondLevel() == rampPosition && 
                                i->getDirectionInSecondLevel() == rampDirection;
        if (isFirstPosition || isSecondPosition) return true;
    }
    return false;
}

MazePosition MazeMap::positionAfterUsingRamp(MazePosition fromPosition, GlobalDirections fromDirection)
{
    for (auto i = ramps.begin(); i != ramps.end(); i++)
    {
        bool isFirstPosition = i->getPositionInFirstLevel() == fromPosition && 
                               i->getDirectionInFirstLevel() == fromDirection;
        if (isFirstPosition) 
            return i->getPositionInFirstLevel();

        bool isSecondPosition = i->getPositionInSecondLevel() == fromPosition && 
                                i->getDirectionInSecondLevel() == fromDirection;
        if (isFirstPosition) 
            return i->getPositionInSecondLevel();
    }
    return fromPosition;
}

void MazeMap::checkpointData()
{
    uncheckpointedTiles.clear();
    
    for (auto i = ramps.begin(); i != ramps.end(); i++)
    {
        i->setAsCheckpointed();
    }
}

void MazeMap::resetSinceLastCheckpoint()
{
    eraseUncheckpointedRamps();
    resetUncheckpointedTiles();
}

void MazeMap::resetUncheckpointedTiles()
{
    for (auto i = uncheckpointedTiles.begin(); i != uncheckpointedTiles.end(); i++)
    {
        mazeLevels[i->levelIndex].resetTileAt(*i);
    }
}

void MazeMap::eraseUncheckpointedRamps()
{
    for (auto i = ramps.begin(); i != ramps.end(); i++)
    {
        if (!i->isCheckpointed())
        {
            ramps.erase(i);
            i--;
        }
    }
}

Ramp MazeMap::getRampFromLevel(int levelIndex) //VERY BAD FUNCION
{
    for (auto i = ramps.begin(); i != ramps.end(); i++)
    {
        if (i->getFirstLevel() == levelIndex)
            return *i;
        if (i->getSecondLevel() == levelIndex)
            return *i;
    }
    return ramps[0];
}