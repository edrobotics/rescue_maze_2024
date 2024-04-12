#pragma once

#include "globalNav/map/directions.h"

struct LevelPosition
{
    int tileX;
    int tileY;
    LevelPosition(int x, int y) : tileX(x), tileY(y) {}
};

struct MazePosition
{
    int tileX;
    int tileY;
    int levelIndex;
    MazePosition(int x, int y, int level) : tileX(x), tileY(y), levelIndex(level) {}
    MazePosition(LevelPosition levelPosition, int level) : tileX(levelPosition.tileY), tileY(levelPosition.tileY), levelIndex(level) {}
    
    operator LevelPosition() {return LevelPosition(tileX, tileY);};

    bool operator ==(MazePosition otherPosition) {
        return this->tileX == otherPosition.tileX && 
               this->tileY == otherPosition.tileY && 
               this->levelIndex == otherPosition.levelIndex;
    };
};

// struct MazePose
// {
//     int tileX;
//     int tileY;
//     int levelIndex;
//     GlobalDirections direction;

//     MazePose(MazePosition levelPosition, GlobalDirections globalDirection) : 
//         tileX(levelPosition.tileY), tileY(levelPosition.tileY), 
//         levelIndex(levelPosition.levelIndex), direction(globalDirection) {}
    
//     operator MazePosition() {return MazePosition(tileX, tileY, levelIndex);};
// };