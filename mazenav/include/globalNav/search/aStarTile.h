#pragma once

#include "globalNav/map/tile.h"
#include "globalNav/map/mazePosition.h"

struct AStarTile
{
    Tile* tile;
    AStarTile* parent;

    MazePosition position;
};