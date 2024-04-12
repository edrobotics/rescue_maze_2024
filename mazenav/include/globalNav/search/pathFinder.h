#pragma once

#include "globalNav/map/mazePath.h"
#include "globalNav/map/position.h"
#include "globalNav/search/aStar.h"

class PathFinder
{
private:
    MazeMap* mazeMap;
public:
    PathFinder(MazeMap* map) : mazeMap(map) {};
    MazePath findPathTo(MazePosition currentPosition, MazePosition toPosition);
};