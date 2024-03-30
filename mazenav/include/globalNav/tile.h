#pragma once

#include <cstdint>

enum class mapBit
{
    front,
    left,
    back,
    right,
    searched,
    inSearch,
    visited,
    explored
};

class Tile
{
    public:
    Tile(uint8_t x, uint8_t y);
    Tile(Tile& t);

    uint8_t x;
    uint8_t y;

    uint16_t g;
    uint16_t h;

    Tile* parent;
    
    void copy(Tile source);
    
    int cost() const { return g + h; };

    uint16_t getInfo();
    inline void setBit(mapBit set, bool state);
    inline bool getBit(mapBit get);
    
    int wallCount();
    
    private:
    uint16_t tileInfo = 0;
};