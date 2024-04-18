#pragma once

#include <vector>
#include <string>
#include "globalNav/map/position.h"

class MazePath
{
private:
    std::vector<MazePosition> positionsInPath;
public:
    MazePosition getNextPosition();
    MazePosition peekTopPosition();
    MazePosition peekBottomPosition();
    void addPositionOnTop(MazePosition position);
    bool isEmpty();
    std::size_t getPositionAmount() const { return positionsInPath.size(); };  
    
    std::string toLoggable();
};