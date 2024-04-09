#include "globalNav/map/mazePath.h"

MazePosition MazePath::getNextPosition()
{
    MazePosition nextPosition = positionsInPath.top();
    positionsInPath.pop();
    return nextPosition;
}

void MazePath::addPosition(MazePosition position)
{
    positionsInPath.push(position);
}

bool MazePath::isEmpty()
{
    return positionsInPath.empty();
}