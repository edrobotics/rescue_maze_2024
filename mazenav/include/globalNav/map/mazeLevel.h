#pragma once

#include "globalNav/map/tile.h"

#define LEVELSIZE 50

class MazeLevel
{
private:
    Tile mazeLevel[LEVELSIZE][LEVELSIZE];
public:
    inline Tile& tileAt(int x, int y) { return mazeLevel[y][x]; };
};