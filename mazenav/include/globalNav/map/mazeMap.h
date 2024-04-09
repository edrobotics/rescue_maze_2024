#pragma once

#include <vector>

#include "globalNav/map/mazeLevel.h"
#include "globalNav/map/levelPosition.h"
#include "globalNav/map/mazePosition.h"
#include "globalNav/map/directions.h"

class MazeMap
{
private:
    std::vector<MazeLevel> mazeLevels;
    Tile::TileProperty directionToWallProperty(Directions direction);
    MazePosition neighborInDirection(MazePosition basePosition, Directions stepDirection);
    bool MazeMap::neighborIsAvailableAndUnexplored(MazePosition position, Directions neighborDirection);
    bool MazeMap::tileHasWallInDirection(MazePosition position, Directions directionToNeighbor);
    
public:
    MazeMap(LevelPosition startPosition);
    bool availableNeighborTilesAreExplored(MazePosition position);
    bool tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty);
    bool MazeMap::neighborHasProperty(MazePosition basePosition, Directions neighborDirection, Tile::TileProperty tileProperty);
    void setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState);
};