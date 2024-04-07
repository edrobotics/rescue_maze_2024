#include "globalNav/map/mazePath.h"

MazePosition MazePath::getNextPosition()
{
    MazePosition nextPosition = positionsInPath.front();
    positionsInPath.erase(positionsInPath.begin());
    return nextPosition;
}

void MazePath::addPosition(MazePosition position)
{
    positionsInPath.push_back(position);
}