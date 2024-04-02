#pragma once

#include "globalNav/map/tile.h"

class AStarTile
{
public:
private:
    Tile* tile;
    AStarTile* parent;

    struct MapPosition
    {
        int xIndex;
        int yIndex;
        int mapIndex;
        MapPosition(int x, int y, int map) : xIndex(x), yIndex(y), mapIndex(map) {}
    };

    MapPosition mapPosition;
};