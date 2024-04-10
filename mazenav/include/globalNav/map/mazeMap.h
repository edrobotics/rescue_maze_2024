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
    bool MazeMap::tileHasWallInDirection(MazePosition position, GlobalDirections directionToNeighbor);
    bool neighborIsInDirection(MazePosition basePosition, MazePosition neighborPosition, GlobalDirections direction);
    
public:
    MazeMap(LevelPosition startPosition);

    bool availableNeighborTilesAreExplored(MazePosition position);
    bool MazeMap::neighborIsAvailableAndUnexplored(MazePosition position, GlobalDirections neighborDirection);

    bool tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty);
    bool MazeMap::neighborHasProperty(MazePosition basePosition, GlobalDirections neighborDirection, Tile::TileProperty tileProperty);
    void setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState);
    
    std::optional<MazePosition> neighborInDirection(MazePosition basePosition, GlobalDirections stepDirection);
    std::optional<GlobalDirections> MazeMap::neighborToDirection(MazePosition basePosition, MazePosition neighborPosition);
};