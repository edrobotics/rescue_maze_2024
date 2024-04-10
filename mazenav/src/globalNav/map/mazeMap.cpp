#include "globalNav/map/mazeMap.h"

MazeMap::MazeMap(LevelPosition startPosition)
{ 
    mazeLevels.push_back(MazeLevel(startPosition));
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
    bool available = tileHasWallInDirection(position, neighborDirection);
    bool explored = neighborHasProperty(position, neighborDirection, Tile::TileProperty::explored);
    return available && !explored;
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
    auto optionalNeighborInDirection = neighborInDirection(basePosition, neighborDirection);
    if (optionalNeighborInDirection)
        return mazeLevels[basePosition.levelIndex].tileHasProperty(optionalNeighborInDirection.value(), tileProperty);
    else
        return false;
}

void MazeMap::setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState)
{
    mazeLevels[tilePosition.levelIndex].setTilePropertyAt(tilePosition, tileProperty, toState);
}


Tile::TileProperty MazeMap::directionToWallProperty(GlobalDirections direction)
{
    switch (direction) // Apologizing for switch, but I could not find a better way
    {
    case GlobalDirections::North:
        return Tile::TileProperty::wallNorth;
    case GlobalDirections::West:
        return Tile::TileProperty::wallWest;
    case GlobalDirections::South:
        return Tile::TileProperty::wallSouth;
    default:
        return Tile::TileProperty::wallEast;
    }
}

std::optional<MazePosition> MazeMap::neighborInDirection(MazePosition basePosition, GlobalDirections neighborDirection)
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
        return std::nullopt;
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
    auto optionalNeighbor = neighborInDirection(basePosition, direction);
    if (optionalNeighbor)
    {
        return optionalNeighbor.value() == neighborPosition;
    }
    return false;
}