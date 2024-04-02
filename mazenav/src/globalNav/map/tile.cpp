#include "globalNav/map/tile.h"

inline void Tile::setTileProperty(TileProperty propertyToSet, bool toState)
{
	tileInfoIntType bitToRead = (0b1 << (tileInfoIntType)propertyToSet); //Shift left to the bit which should be read

    if (toState) {
        tileInfo = tileInfo | (bitToRead); //Sets bit to true
    } else {
        tileInfo = tileInfo & ~bitToRead; //Sets bit to false
    }
}

inline bool Tile::tileHasProperty(TileProperty propertyToGet)
{
	return ((tileInfo >> (tileInfoIntType)propertyToGet) & 0b1);
}