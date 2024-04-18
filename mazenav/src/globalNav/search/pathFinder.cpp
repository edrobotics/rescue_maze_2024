#include "globalNav/search/pathFinder.h"

MazePath PathFinder::findPathTo(MazePosition currentPosition, MazePosition toPosition)
{
    if (currentPosition.levelIndex == toPosition.levelIndex)
    {
        AStar aStarSearchAlgorithm(currentPosition, toPosition, mazeMap);
        return aStarSearchAlgorithm.getAStarResult();
    }
    else //DELETE
    {
        Ramp rampToUse = mazeMap->getRampFromLevel(currentPosition.levelIndex);
        MazePosition rampPositionInCurrentLevel = rampToUse.getPositionInLevel(currentPosition.levelIndex).value();
        AStar aStarSearchToRamp(currentPosition, rampPositionInCurrentLevel, mazeMap);
        AStar aStarSearchFromRamp(rampToUse.getPositionInLevel(toPosition.levelIndex).value(), rampPositionInCurrentLevel, mazeMap);

        return combineMazePathsAndAddRampTile(aStarSearchToRamp.getAStarResult(), aStarSearchFromRamp.getAStarResult(), currentPosition.levelIndex);
    }
}

MazePath PathFinder::combineMazePathsAndAddRampTile(MazePath beforeRamp, MazePath afterRamp, int beforeRampLevelIndex)
{
    Ramp rampToUse = mazeMap->getRampFromLevel(beforeRampLevelIndex);
    GlobalDirections directionOfRamp;
    MazePosition positionBeforeRamp = beforeRamp.peekBottomPosition();

    if (rampToUse.getFirstLevel() == beforeRampLevelIndex)
        directionOfRamp = rampToUse.getDirectionInFirstLevel();
    else
        directionOfRamp = rampToUse.getDirectionInSecondLevel();

    afterRamp.addPositionOnTop(mazeMap->neighborInDirection(positionBeforeRamp, directionOfRamp));

    while (!beforeRamp.isEmpty())
    {
        afterRamp.addPositionOnTop(beforeRamp.getNextPosition());
    }
    return afterRamp;
}