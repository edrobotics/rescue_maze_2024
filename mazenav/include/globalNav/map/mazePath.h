#pragma once

#include <vector>
#include "globalNav/map/mazePosition.h"

class MazePath
{
public:
    MazePosition getNextPosition();
    void addPosition(MazePosition position);
private:
    std::vector<MazePosition> positionsInPath;
};