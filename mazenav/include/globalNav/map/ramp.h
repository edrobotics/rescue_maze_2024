#pragma once

#include "globalNav/map/position.h"
#include "globalNav/map/directions.h"

class Ramp
{
    private:
    MazePosition firstLevelPosition;
    GlobalDirections firstLevelDirection;
    MazePosition secondLevelPosition;
    GlobalDirections secondLevelDirection;
    public:
    Ramp(MazePosition positionInFirstLevel, MazePosition positionInSecondLevel, GlobalDirections directionFromFirstLevel);
    
    MazePosition getPositionInFirstLevel() const {return firstLevelPosition;};
    GlobalDirections getDirectionInFirstLevel() const {return firstLevelDirection;};

    MazePosition getPositionInSecondLevel() const {return secondLevelPosition;};
    GlobalDirections getDirectionInSecondLevel() const {return secondLevelDirection;};
};