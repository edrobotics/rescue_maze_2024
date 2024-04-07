#pragma once

#include "globalNav/map/tile.h"
#include "globalNav/map/mazePosition.h"

class AStarTile
{
public:
private:
    Tile* tile;
    AStarTile* parent;

    MazePosition position;
};