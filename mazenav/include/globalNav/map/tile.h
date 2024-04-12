#pragma once

#include <cstdint>

class Tile
{
    public:

    enum class TileProperty
    {
        WallNorth,
        WallWest,
        WallSouth,
        WallEast,
        Explored,
        SearchAlgorithmVisited,
        Checkpoint,
        Black,
        Blue,
        ContainsRamp
    };

    void setTileProperty(TileProperty set, bool state);
    bool tileHasProperty(TileProperty get);
    
    private:
    typedef int16_t tileInfoIntType;
    tileInfoIntType tileInfo = 0;
};