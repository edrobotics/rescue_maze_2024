#include "globalNav/map/ramp.h"

Ramp::Ramp(MazePosition fromPosition, MazePosition toPosition, GlobalDirections travelDirection)
: firstLevelPosition(fromPosition), secondLevelPosition(toPosition)
{
    firstLevelDirection = travelDirection;
    int reverseFirstDirectionInt = ((int)travelDirection + DIRECTIONS_AMOUNT/2) % DIRECTIONS_AMOUNT;
    secondLevelDirection = (GlobalDirections)reverseFirstDirectionInt;
};