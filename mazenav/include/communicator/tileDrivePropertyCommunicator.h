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

        // True if data should be filled by fusion (poseEstimator)
        bool readyForDataFill {false};
        // Set to true when starting driving
        bool driveStarted {false};

        std::mutex mtx_tileInfo;
        public:

        void setNewTileProperties(TileDriveProperties tileProperties);
        TileDriveProperties readLatestTileProperties(); //Call when drive is finished, create vector with properties of the new tile, e.g. checkpoint

        bool hasNewTileInfo();

        void setReadyForFill();
        bool getIsReadyForFill();

        void startDrive();
        // Returns true if drive was started. Also resets the flag => will return false next time. Be cautious about calling multiple times.
        bool getDriveStarted();
    };
}