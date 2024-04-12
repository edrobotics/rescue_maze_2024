#pragma once

#include "globalNav/map/tile.h"
#include "globalNav/map/position.h"

struct AStarTile
{
    AStarTile* parent = nullptr;

    MazePosition position;

    int h = 0;
    int g = 0;

    int totalCost() const {return h + g;}
    AStarTile(MazePosition mazePosition) : position(mazePosition) {};
};