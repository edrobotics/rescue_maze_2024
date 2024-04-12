#pragma once

#include "globalNav/map/tile.h"
#include "globalNav/map/position.h"

#define LEVELSIZE 50

class MazeLevel
{
private:
    Tile mazeLevel[LEVELSIZE][LEVELSIZE] = {{Tile()}};
public:
    void setTilePropertyAt(LevelPosition position, Tile::TileProperty property, bool toState);
    bool tileHasProperty(LevelPosition tilePosition, Tile::TileProperty property);
    inline Tile& tileAt(int x, int y) { return mazeLevel[y][x]; };
};