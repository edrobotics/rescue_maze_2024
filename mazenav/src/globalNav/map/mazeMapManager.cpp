#include "globalNav/map/mazeMapManager.h"

MazeMapManager::MazeMapManager(MapPosition startPosition) : currentMapPosition(startPosition) 
{ 
    mazeMaps.push_back(MazeMap());
};