#include "globalNav/map/mazeLevel.h"

void MazeLevel::setTilePropertyAt(LevelPosition position, Tile::TileProperty property, bool toState)
{
    mazeLevel[position.tileY][position.tileX].setTileProperty(property, toState);
}

bool MazeLevel::tileHasProperty(LevelPosition tilePosition, Tile::TileProperty property)
{
    return mazeLevel[tilePosition.tileY][tilePosition.tileX].tileHasProperty(property);
}