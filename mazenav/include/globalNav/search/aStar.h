#pragma once

#include <queue>
#include <vector>
#include <math.h>

#include "globalNav/search/aStarTile.h"
#include "globalNav/map/position.h"
#include "globalNav/map/mazeMap.h"
#include "globalNav/map/mazePath.h"

class AStar
{
private:
    MazePosition startPosition;
    MazePosition endPosition;
    MazeMap* mazeMap;

    MazePath foundPath;

    struct CompareWeights
	{
		bool operator()(const AStarTile* left, const AStarTile* right) const 
		{
			return ((left->totalCost()) >( right->totalCost())); //compare tile distances
		}
	};

    std::priority_queue<AStarTile*, std::vector<AStarTile*>, AStar::CompareWeights> aStarTileQueue;
    std::vector<AStarTile*> toDelete;

    void aStarSearch();
    AStarTile* getBestTile();
    void constructPath(AStarTile* finalTile);
    void expandTile(AStarTile* tile);
    void storeAndAddTileToQueue(AStarTile* tile);
    AStarTile* createTile(MazePosition withPosition);

    AStarTile* createTileWithProperties(MazePosition newPosition, AStarTile* parentTile);
    int heuristic(MazePosition fromPosition);

    bool neighborIsAvailable(MazePosition tile, MazePosition neighbor, GlobalDirections neighborDirection);
    bool inBounds(MazePosition position);

public:
    AStar(MazePosition currentPosition, MazePosition toPosition, MazeMap* map);
    ~AStar();
    MazePath getAStarResult();
};