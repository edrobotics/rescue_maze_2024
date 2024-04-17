#pragma once

#include <optional>
#include "globalNav/map/position.h"
#include "globalNav/map/directions.h"

class Ramp
{
    private:
    MazePosition firstLevelPosition;
    GlobalDirections firstLevelDirection;
    MazePosition secondLevelPosition;
    GlobalDirections secondLevelDirection;
    bool hasBeenCheckpointed = false;
    public:
    Ramp(MazePosition positionInFirstLevel, MazePosition positionInSecondLevel, GlobalDirections directionFromFirstLevel);
    
    MazePosition getPositionInFirstLevel() const {return firstLevelPosition;};
    GlobalDirections getDirectionInFirstLevel() const {return firstLevelDirection;};
    int getFirstLevel() const { return firstLevelPosition.levelIndex; };

    MazePosition getPositionInSecondLevel() const {return secondLevelPosition;};
    GlobalDirections getDirectionInSecondLevel() const {return secondLevelDirection;};
    int getSecondLevel() const { return secondLevelPosition.levelIndex; };


    void setAsCheckpointed() { hasBeenCheckpointed = true; };
    bool isCheckpointed() const { return hasBeenCheckpointed; };

    std::optional<MazePosition> getPositionInLevel(int levelIndex);
};