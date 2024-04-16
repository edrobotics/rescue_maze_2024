#pragma once

#include <stack>
#include "globalNav/map/position.h"

class MazePath
{
private:
    std::stack<MazePosition> positionsInPath;
public:
    MazePosition getNextPosition();
    void addPositionInFront(MazePosition position);
    bool isEmpty();
    std::size_t getPositionAmount() const { return positionsInPath.size(); };
};