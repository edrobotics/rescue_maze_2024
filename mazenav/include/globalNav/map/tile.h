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
        ContainsRamp,
        HasVictim,
    };

    void setTileProperty(TileProperty set, bool state);
    bool tileHasProperty(TileProperty get);

    void resetTileExceptVictims();
    
    private:
    typedef int16_t tileInfoIntType;
    tileInfoIntType tileInfo = 0;
};