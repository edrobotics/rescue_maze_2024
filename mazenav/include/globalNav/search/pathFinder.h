#pragma once

#include "globalNav/map/mazePath.h"
#include "globalNav/map/position.h"
#include "globalNav/search/aStar.h"

class PathFinder
{
private:
    MazeMap* mazeMap;
    MazePath combineMazePathsAndAddRampTile(MazePath beforeRamp, MazePath afterRamp, int beforeRampLevelIndex);
public:
    PathFinder(MazeMap* map) : mazeMap(map) {};
    MazePath findPathTo(MazePosition currentPosition, MazePosition toPosition);
};