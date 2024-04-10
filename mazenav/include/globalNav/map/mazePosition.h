#pragma once

#include "globalNav/map/levelPosition.h"

struct MazePosition
{
    int tileX;
    int tileY;
    int levelIndex;
    MazePosition(int x, int y, int map) : tileX(x), tileY(y), levelIndex(map) {}
    MazePosition(LevelPosition levelPosition, int map) : tileX(levelPosition.tileY), tileY(levelPosition.tileY), levelIndex(map) {}
    
    operator LevelPosition() {return LevelPosition(tileX, tileY);};
    bool operator ==(MazePosition otherPosition) {
        return this->tileX == otherPosition.tileX && 
               this->tileY == otherPosition.tileY && 
               this->levelIndex == otherPosition.levelIndex;
    };
};
