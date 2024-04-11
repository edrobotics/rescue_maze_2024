#include "globalNav/search/aStar.h"

AStar::AStar(MazePosition currentPosition, MazePosition toPosition, MazeMap* map) : startPosition(currentPosition), endPosition(toPosition), mazeMap(map)
{
    aStarSearch();
}

MazePath AStar::getAStarResult()
{
    return foundPath;
}

void AStar::aStarSearch()
{
    AStarTile startTile(startPosition);
    startTile.g = 0;
    storeAndAddTileToQueue(startTile);

    while (!aStarTileQueue.empty())
    {
        AStarTile* currentTile = getBestTile();

        if (currentTile->position == endPosition) 
		{
			constructPath(currentTile);
			return;
		}

        expandTile(currentTile);
    }
}

AStarTile* AStar::getBestTile()
{
    AStarTile* bestTile = aStarTileQueue.top();
    aStarTileQueue.pop();
    return bestTile;
}

void AStar::constructPath(AStarTile* finalTile)
{
    AStarTile* currentTile = finalTile;
    while (currentTile != nullptr) 
	{
		foundPath.addPositionInFront(currentTile->position);
		currentTile = currentTile->parent;
	}
}

void AStar::expandTile(AStarTile* tile)
{
    for (int direction = 0; direction < 4; direction++) 
    {
        GlobalDirections neighborDirection = (GlobalDirections)direction;
        MazePosition newPosition = mazeMap->neighborInDirection(tile->position, neighborDirection);
		
        if (neighborIsAvailable(tile->position, newPosition, neighborDirection))
            setTileProperties(newPosition, tile);
    }
}

AStarTile AStar::setTileProperties(MazePosition newPosition, AStarTile* parentTile)
{
    AStarTile newTile(newTile);

	newTile.g = parentTile->g + 1;
	newTile.h = heuristic(newPosition);
	newTile.parent = parentTile;
	mazeMap->setTileProperty(newPosition, Tile::TileProperty::SearchAlgorithmVisited, true);

    return newTile;
}

int AStar::heuristic(MazePosition fromPosition)
{
    return std::abs(fromPosition.tileX - endPosition.tileX) + std::abs(fromPosition.tileY - endPosition.tileY);
}

void AStar::storeAndAddTileToQueue(AStarTile tile)
{
    storageVector.push_back(tile);
    aStarTileQueue.push(&storageVector.back());
}

bool AStar::neighborIsAvailable(MazePosition tile, MazePosition neighbor, GlobalDirections neighborDirection)
{
   return mazeMap->neighborIsAvailableAndUnexplored(tile, neighborDirection) && 
   !mazeMap->tileHasProperty(neighbor, Tile::TileProperty::SearchAlgorithmVisited) &&
   inBounds(neighbor);
   //inBounds(newPosition) && !maze[newX][newY].getBit(mBit::visited) && !currentTile->getBit((mBit)direction);
}

bool AStar::inBounds(MazePosition position)
{
    if (position.tileX > LEVELSIZE || position.tileX < 0) return false;
    if (position.tileY > LEVELSIZE || position.tileY < 0) return false;
    return true;
}