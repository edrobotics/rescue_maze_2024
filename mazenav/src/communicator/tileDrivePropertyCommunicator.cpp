#include "communicator/tileDrivePropertyCommunicator.h"

namespace communication
{
    void TileDrivePropertyCommunicator::setNewTileProperties(TileDriveProperties tileProperties)
    {
        mtx_tileInfo.lock();
        
        tileIsUnread = true;
        latestTileProperties = tileProperties;

        readyForDataFill = false;

        mtx_tileInfo.unlock();
    }

    TileDriveProperties TileDrivePropertyCommunicator::readLatestTileProperties()
    {
        mtx_tileInfo.lock();

        tileIsUnread = false;
        TileDriveProperties properties = latestTileProperties;

        mtx_tileInfo.unlock();

        return properties;
    }


    bool TileDrivePropertyCommunicator::hasNewTileInfo()
    {
        return tileIsUnread;
    }

    void TileDrivePropertyCommunicator::setReadyForFill()
    {
        mtx_tileInfo.lock();
        readyForDataFill = true;
        mtx_tileInfo.unlock();
    }

    bool TileDrivePropertyCommunicator::getIsReadyForFill()
    {
        std::lock_guard<std::mutex> lock(mtx_tileInfo);
        return readyForDataFill;
    }

    void TileDrivePropertyCommunicator::startDrive()
    {
        std::lock_guard<std::mutex> lock(mtx_tileInfo);
        driveStarted = true;
    }

    bool TileDrivePropertyCommunicator::getDriveStarted()
    {
        std::lock_guard<std::mutex> lock(mtx_tileInfo);
        bool retVal {driveStarted};
        if (driveStarted==true)
        {
            driveStarted = false;
        }
        return retVal;
    }
}