#include "globalNav/search/pathFinder.h"

MazePath PathFinder::findPathTo(MazePosition currentPosition, MazePosition toPosition)
{
    AStar aStarSearchAlgorithm(currentPosition, toPosition, mazeMap);
    return aStarSearchAlgorithm.getAStarResult();
}


//BFS expansion to get closest knownUnexploredTile and path to it