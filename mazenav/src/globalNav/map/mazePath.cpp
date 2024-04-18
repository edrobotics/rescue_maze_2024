#include "globalNav/map/mazePath.h"

MazePosition MazePath::getNextPosition()
{
    MazePosition nextPosition = positionsInPath.back();
    positionsInPath.pop_back();
    return nextPosition;
}

MazePosition MazePath::peekTopPosition()
{
    return positionsInPath.back();
}

MazePosition MazePath::peekBottomPosition()
{
    return positionsInPath.front();
}

void MazePath::addPositionOnTop(MazePosition position)
{
    positionsInPath.push_back(position);
}

bool MazePath::isEmpty()
{
    return positionsInPath.empty();
}

std::string MazePath::toLoggable()
{
    if (positionsInPath.empty()) return "Path is empty";

    std::string baseString = "PATH FROM " + positionsInPath.back().toLoggable() + " TO " + positionsInPath.front().toLoggable() + " : ";

    for (MazePosition& position : positionsInPath)
    {
        baseString += position.toLoggable();
    }
    return baseString;
}