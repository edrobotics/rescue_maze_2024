#pragma once

#include <vector>
#include <optional>

#include "globalNav/map/mazeLevel.h"
#include "globalNav/map/position.h"
#include "globalNav/map/directions.h"
#include "globalNav/map/ramp.h"

class MazeMap
{
private:
    std::vector<MazeLevel> mazeLevels;
    std::vector<Ramp> ramps;
    Tile::TileProperty directionToWallProperty(GlobalDirections direction);
    bool tileHasWallInDirection(MazePosition position, GlobalDirections directionToNeighbor);
    bool neighborIsInDirection(MazePosition basePosition, MazePosition neighborPosition, GlobalDirections direction);

    void createNewLevel();
    
public:
    MazeMap();

    bool availableNeighborTilesAreExplored(MazePosition position);
    bool neighborIsAvailableAndUnexplored(MazePosition position, GlobalDirections neighborDirection);
    bool neighborIsAvailable(MazePosition position, GlobalDirections neighborDirection);

    bool tileHasProperty(MazePosition tilePosition, Tile::TileProperty tileProperty);
    bool neighborHasProperty(MazePosition basePosition, GlobalDirections neighborDirection, Tile::TileProperty tileProperty);
    void setTileProperty(MazePosition tilePosition, Tile::TileProperty tileProperty, bool toState);
    
    MazePosition neighborInDirection(MazePosition basePosition, GlobalDirections stepDirection);
    std::optional<GlobalDirections> neighborToDirection(MazePosition basePosition, MazePosition neighborPosition);

    void createNewRamp(MazePosition previousPosition, MazePosition newPosition, GlobalDirections direction);
    bool canCreateNewRamps();
    bool rampHasBeenUsedBefore(MazePosition rampPosition, GlobalDirections rampDirection);
    MazePosition positionAfterUsingRamp(MazePosition fromPosition, GlobalDirections fromDirection);
};