#pragma once

#include <cstdint>
#include <communicator/communicator.h>

using namespace std;

namespace globalNav
{
    enum mBit
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

    class tile
    {
        public:
        uint8_t x;
        uint8_t y;

        tile(uint8_t x, uint8_t y);
        tile(tile* t);
        void copy(tile source);

        uint16_t g;
        uint16_t h;

        int cost() const { return g + h; };
        tile* parent;

        uint16_t getInfo();
        void setBit(mBit set, bool state);
        bool getBit(mBit get);
        int wallCount();
        
        private:
        uint16_t info;
    };

    void main(communication::Communicator communicator);
}

