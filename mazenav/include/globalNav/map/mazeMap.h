#pragma once

#include "globalNav/map/mazeLevel.h"
#include "globalNav/map/levelPosition.h"
#include "globalNav/map/mazePosition.h"
#include <vector>

class MazeMap
{
private:
    std::vector<MazeLevel> mazeMaps;
public:
    MazeMap();
};