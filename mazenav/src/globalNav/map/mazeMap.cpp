#include "globalNav/map/mazeMap.h"

MazeMap::MazeMap(LevelPosition startPosition)
{ 
    mazeLevels.push_back(MazeLevel(startPosition));
};

bool MazeMap::availableNeighborTilesAreExplored(MazePosition position)
{
    bool northAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, Directions::North);
    bool westAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, Directions::West);
    bool southAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, Directions::South);
    bool eastAvailableAndUnexplored = neighborIsAvailableAndUnexplored(position, Directions::East);
    return northAvailableAndUnexplored || westAvailableAndUnexplored || southAvailableAndUnexplored || eastAvailableAndUnexplored;
}

bool MazeMap::neighborIsAvailableAndUnexplored(MazePosition position, Directions neighborDirection)
{
    bool available = tileHasWallInDirection(position, neighborDirection);
    bool explored = neighborHasProperty(position, neighborDirection, Tile::TileProperty::explored);
    return available && !explored;
}

bool MazeMap::tileHasWallInDirection(MazePosition position, Directions wallDirection)
{
    return tileHasProperty(position, directionToWallProperty(wallDirection));
}

bool MazeMap::tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty)
{
    return mazeLevels[tilePosition.levelIndex].tileHasProperty(tilePosition, tileProperty);
}

bool MazeMap::neighborHasProperty(MazePosition basePosition, Directions neighborDirection, Tile::TileProperty tileProperty)
{
    return mazeLevels[basePosition.levelIndex].tileHasProperty(neighborInDirection(basePosition, neighborDirection), tileProperty);
}

void MazeMap::setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState)
{
    mazeLevels[tilePosition.levelIndex].setTilePropertyAt(tilePosition, tileProperty, toState);
}


Tile::TileProperty MazeMap::directionToWallProperty(Directions direction)
{
    switch (direction) // Apologizing for switch, but I could not find a better way
    {
    case Directions::North:
        return Tile::TileProperty::wallNorth;
    case Directions::West:
        return Tile::TileProperty::wallWest;
    case Directions::South:
        return Tile::TileProperty::wallSouth;
    default:
        return Tile::TileProperty::wallEast;
    }
}

MazePosition MazeMap::neighborInDirection(MazePosition basePosition, Directions neighborDirection)
{
    int xOffset = 0, yOffset = 0;
    switch (neighborDirection) // Apologizing for switch, but I could not find a better way
    {
    case Directions::North: 
        yOffset = -1;
        break;
    case Directions::West: 
        xOffset = -1;
        break;
    case Directions::South: 
        yOffset = 1;
        break;
    case Directions::East: 
        xOffset = 1;
        break;
    default:
        break;
    }

    return MazePosition(basePosition.tileX + xOffset, basePosition.tileY + yOffset, basePosition.levelIndex);
}