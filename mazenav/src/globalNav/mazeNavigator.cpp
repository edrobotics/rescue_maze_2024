#include "globalNav/mazeNavigator.h"

void MazeNavigator::makeNavigationDecision()
{
    if (tilesInPath()) 
        followPath();
    else
        exploreMaze();
}

bool MazeNavigator::tilesInPath()
{
    return !pathToFollow.isEmpty();
}

void MazeNavigator::followPath()
{

    // pathToFollow.getNextPosition();
}

void MazeNavigator::exploreMaze()
{
    // NeighborTiles neighborTiles = mazeMap.getNeighboringTiles(currentPosition);
    if (mazeMap.availableNeighborTilesAreExplored(currentPosition))
    {
        pathToFollow = pathTo(knownUnexploredTilePositions.back());
        knownUnexploredTilePositions.pop_back();
    }
    else
    {
        exploreBestNeighbor();
    }
}

void MazeNavigator::startFollowingPathToLastUnexploredTile()
{
    pathToFollow = pathTo(knownUnexploredTilePositions.back());
    knownUnexploredTilePositions.pop_back();

    followPath();
}

MazePath MazeNavigator::pathTo(MazePosition toPosition)
{
    pathFinder.findPathTo(toPosition);
}

void MazeNavigator::exploreBestNeighbor()
{
    #warning Method not done
}