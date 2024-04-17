#include "globalNav/search/pathFinder.h"

MazePath PathFinder::findPathTo(MazePosition currentPosition, MazePosition toPosition)
{
    // if (currentPosition.levelIndex == toPosition.levelIndex)
    // {
        AStar aStarSearchAlgorithm(currentPosition, toPosition, mazeMap);
        return aStarSearchAlgorithm.getAStarResult();
    // }
    // else //DELETE
    // {
    //     Ramp rampToUse = mazeMap->getRampFromLevel(currentPosition.levelIndex);
    //     MazePosition rampPositionInCurrentLevel = rampToUse.getPositionInLevel(currentPosition.levelIndex).value();
    //     AStar aStarSearchToRamp(currentPosition, rampPositionInCurrentLevel, mazeMap);
    //     AStar aStarSearchFromRamp(rampToUse.getPositionInLevel(toPosition.levelIndex).value(), rampPositionInCurrentLevel, mazeMap);
    //     #error Combine paths and return
    // }
}


//BFS expansion to get closest unexplored tile and path to it