#include "communicator/tileDrivePropertyCommunicator.h"

namespace communication
{
    void TileDrivePropertyCommunicator::setNewTileProperties(TileDriveProperties tileProperties)
    {
        mtx_tileInfo.lock();
        
        tileIsUnread = true;
        latestTileProperties = tileProperties;

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
}