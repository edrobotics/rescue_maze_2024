#include "globalNav/map/tile.h"

void Tile::setTileProperty(TileProperty propertyToSet, bool toState)
{
	tileInfoIntType bitToRead = (0b1 << (tileInfoIntType)propertyToSet); //Shift left to the bit which should be read

    if (toState) {
        tileInfo = tileInfo | (bitToRead); //Sets bit to true
    } else {
        tileInfo = tileInfo & ~bitToRead; //Sets bit to false
    }
}

bool Tile::tileHasProperty(TileProperty propertyToGet)
{
	return ((tileInfo >> (tileInfoIntType)propertyToGet) & 0b1);
}

void Tile::resetTileExceptVictims()
{
    bool hasVictim = tileHasProperty(TileProperty::HasVictim); //The reason we save victims is because re-detecting victims is bad

    tileInfo = 0;

    setTileProperty(TileProperty::HasVictim, hasVictim);
}