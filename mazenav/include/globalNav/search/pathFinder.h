#pragma once

#include "globalNav/map/mazePath.h"
#include "globalNav/map/mazePosition.h"
#include "globalNav/search/aStar.h"

class PathFinder
{
private:
public:
    MazePath findPathTo(MazePosition toPosition);
};