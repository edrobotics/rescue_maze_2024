#pragma once

#include <vector>
#include <mutex>
#include "fusion/ColourSensor.h"

namespace communication
{
    enum class Walls
    {
        FrontWall,
        LeftWall,
        BackWall,
        RightWall
    };

    struct TileDriveProperties
    {
        std::vector<Walls> wallsOnNewTile;
        TileColours tileColourOnNewTile;
        
        bool droveTile;
        bool usedRamp;
    };

    class TileDrivePropertyCommunicator
    {
        private:
        TileDriveProperties latestTileProperties;
        bool tileIsUnread = false;

        std::mutex mtx_tileInfo;
        public:

        void setNewTileProperties(TileDriveProperties tileProperties);
        TileDriveProperties readLatestTileProperties(); //Call when drive is finished, create vector with properties of the new tile, e.g. checkpoint

        bool hasNewTileInfo();
    };
}