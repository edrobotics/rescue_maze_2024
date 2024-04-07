#pragma once

#include <cstdint>

class Tile
{
    public:

    enum class TileProperty
    {
        wallNorth,
        wallWest,
        wallSouth,
        wallEast,
        uncertainWallNorth,
        uncertainWallWest,
        uncertainWallSouth,
        uncertainWallEast,
        explored
    };

    inline void setTileProperty(TileProperty set, bool state);
    inline bool tileHasProperty(TileProperty get);
    
    private:
    typedef int32_t tileInfoIntType;
    tileInfoIntType tileInfo = 0;
};