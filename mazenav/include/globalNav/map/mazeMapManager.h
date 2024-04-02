#pragma once

#include "globalNav/map/mazeMap.h"
#include "globalNav/map/mapPosition.h"
#include <vector>

class MazeMapManager
{
private:
    std::vector<MazeMap> mazeMaps;
    MapPosition currentMapPosition = MapPosition(MAZESIZE/2, MAZESIZE/2, 0);
public:
    MazeMapManager(MapPosition startPosition);
};