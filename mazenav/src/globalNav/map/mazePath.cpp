#include "globalNav/map/mazePath.h"

MazePosition MazePath::getNextPosition()
{
    MazePosition nextPosition = positionsInPath.top();
    positionsInPath.pop();
    return nextPosition;
}

void MazePath::addPositionInFront(MazePosition position)
{
    positionsInPath.push(position);
}

bool MazePath::isEmpty()
{
    return positionsInPath.empty();
}