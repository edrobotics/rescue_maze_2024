#pragma once

#include <vector>
#include <optional>

#include "globalNav/map/mazeLevel.h"
#include "globalNav/map/levelPosition.h"
#include "globalNav/map/mazePosition.h"
#include "globalNav/map/directions.h"

class MazeMap
{
private:
    std::vector<MazeLevel> mazeLevels;
    Tile::TileProperty directionToWallProperty(GlobalDirections direction);
    bool tileHasWallInDirection(MazePosition position, GlobalDirections directionToNeighbor);
    bool neighborIsInDirection(MazePosition basePosition, MazePosition neighborPosition, GlobalDirections direction);
    
public:
    MazeMap(LevelPosition startPosition);

    bool availableNeighborTilesAreExplored(MazePosition position);
    bool neighborIsAvailableAndUnexplored(MazePosition position, GlobalDirections neighborDirection);
    bool neighborIsAvailable(MazePosition position, GlobalDirections neighborDirection);

    bool tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty);
    bool neighborHasProperty(MazePosition basePosition, GlobalDirections neighborDirection, Tile::TileProperty tileProperty);
    void setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState);
    
    MazePosition neighborInDirection(MazePosition basePosition, GlobalDirections stepDirection);
    std::optional<GlobalDirections> neighborToDirection(MazePosition basePosition, MazePosition neighborPosition);
};