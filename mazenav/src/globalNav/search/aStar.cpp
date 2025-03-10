#include "globalNav/search/aStar.h"

AStar::AStar(MazePosition currentPosition, MazePosition toPosition, MazeMap* map) : 
            startPosition(currentPosition), endPosition(toPosition), mazeMap(map)
{
    aStarSearch();
    cleanUp();
}

void AStar::cleanUp()
{
    while (!toDelete.empty())
    {
        mazeMap->setTileProperty(toDelete.back()->position, Tile::TileProperty::SearchAlgorithmVisited, false);
        delete toDelete.back();
        toDelete.erase(toDelete.end()-1);
    }
}

MazePath AStar::getAStarResult()
{
    return foundPath;
}

void AStar::aStarSearch()
{
    AStarTile* startTile = createTile(startPosition);
    storeAndAddTileToQueue(startTile);

    while (!aStarTileQueue.empty())
    {
        AStarTile* currentTile = getBestTile();
        std::cout << "AS: On tile " << currentTile->position.toLoggable() << std::endl;

        if (currentTile->position == endPosition) 
		{
			constructPath(currentTile);
			return;
		}

        expandTile(currentTile);
    }
    std::cout << "AS: END BAD BAD " << std::endl;

}

AStarTile* AStar::createTile(MazePosition withPosition)
{
    AStarTile* newTile = new AStarTile(withPosition);
    toDelete.push_back(newTile);
    return newTile;
}

AStarTile* AStar::getBestTile()
{
    AStarTile* bestTile = aStarTileQueue.top();
    aStarTileQueue.pop();
    return bestTile;
}

void AStar::constructPath(AStarTile* finalTile)
{
    std::cout << "AS: found way back" << std::endl;
    AStarTile* currentTile = finalTile;
    while (currentTile != nullptr) 
	{
        std::cout << "AS: Add tile back " << currentTile->position.toLoggable() << std::endl;
		foundPath.addPositionOnTop(currentTile->position);
		currentTile = currentTile->parent;
	}
}

void AStar::expandTile(AStarTile* tile)
{
    std::cout << "AS: Expanding tile " << tile->position.toLoggable() << std::endl;
    for (int direction = 0; direction < 4; direction++) 
    {
        GlobalDirections neighborDirection = (GlobalDirections)direction;
        MazePosition newPosition = mazeMap->neighborInDirection(tile->position, neighborDirection);
		
        if (neighborIsAvailableFrom(tile->position, newPosition, neighborDirection))
        {
            std::cout << "AS: Adding tile to queue " << newPosition.toLoggable() << std::endl;
            storeAndAddTileToQueue(createTileWithProperties(newPosition, tile));
        }
    }
}

AStarTile* AStar::createTileWithProperties(MazePosition newPosition, AStarTile* parentTile)
{
    AStarTile* newTile = createTile(newPosition);

	newTile->g = parentTile->g + 1;
	newTile->h = heuristic(newPosition);
	newTile->parent = parentTile;
	mazeMap->setTileProperty(newPosition, Tile::TileProperty::SearchAlgorithmVisited, true);

    return newTile;
}

int AStar::heuristic(MazePosition fromPosition)
{
    return std::abs(fromPosition.tileX - endPosition.tileX) + std::abs(fromPosition.tileY - endPosition.tileY);
}

void AStar::storeAndAddTileToQueue(AStarTile* tile)
{
    aStarTileQueue.push(tile);
}

bool AStar::neighborIsAvailableFrom(MazePosition tile, MazePosition neighbor, GlobalDirections neighborDirection)
{
    if (!inBounds(neighbor)) return false;

    return mazeMap->neighborIsAvailableFrom(tile, neighborDirection) && 
        !mazeMap->tileHasProperty(neighbor, Tile::TileProperty::SearchAlgorithmVisited) &&
        (mazeMap->tileHasProperty(neighbor, Tile::TileProperty::Explored) || neighbor == endPosition);
}

bool AStar::inBounds(MazePosition position)
{
    if (position.tileX > LEVELSIZE || position.tileX < 0) return false;
    if (position.tileY > LEVELSIZE || position.tileY < 0) return false;
    return true;
}