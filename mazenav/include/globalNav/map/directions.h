#pragma once

#define DIRECTIONS_AMOUNT 4

enum class GlobalDirections
{
    North = 0, //-Y
    West = 1, //-X
    South = 2,
    East = 3
}; //MUST BE IN THIS ORDER

enum class LocalDirections
{
    Front = 0,
    Left = 1,
    Back = 2,
    Right = 3
}; //MUST BE IN THIS ORDER