#pragma once

struct MapPosition
{
    int xIndex;
    int yIndex;
    int mapIndex;
    MapPosition(int x, int y, int map) : xIndex(x), yIndex(y), mapIndex(map) {}
};