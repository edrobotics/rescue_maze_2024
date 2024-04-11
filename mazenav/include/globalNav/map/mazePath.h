#pragma once

#include <stack>
#include "globalNav/map/mazePosition.h"

class MazePath
{
public:
    MazePosition getNextPosition();
    void addPositionInFront(MazePosition position);
    bool isEmpty();
private:
    std::stack<MazePosition> positionsInPath;
};