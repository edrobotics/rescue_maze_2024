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
    bool victimFront = tileHasProperty(TileProperty::VictimNorth); //The reason we save victims is because re-detecting victims is bad
    bool victimLeft = tileHasProperty(TileProperty::VictimWest);
    bool victimBack = tileHasProperty(TileProperty::VictimSouth);
    bool victimRight = tileHasProperty(TileProperty::VictimEast);

    tileInfo = 0;

    if (victimFront) setTileProperty(TileProperty::VictimNorth, true);
    if (victimLeft) setTileProperty(TileProperty::VictimWest, true);
    if (victimBack) setTileProperty(TileProperty::VictimSouth, true);
    if (victimRight) setTileProperty(TileProperty::VictimEast, true);
}