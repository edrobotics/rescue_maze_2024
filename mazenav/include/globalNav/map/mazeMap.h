#pragma once

#include "globalNav/map/tile.h"

#define MAZESIZE 50

class MazeMap
{
private:
    Tile mazeMap[MAZESIZE][MAZESIZE];
public:
    MazeMap();
    inline Tile& tileAt(int x, int y) { return mazeMap[y][x]; };
};