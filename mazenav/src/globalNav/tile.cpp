#include <tile.h>

Tile::Tile(uint8_t x, uint8_t y)
{
	this->x = x;
	this->y = y;
}

Tile::Tile(Tile& t)
{
	this->copy(t);
}

void Tile::copy(Tile source)
{
	this->tileInfo = source.getInfo();
	this->x = source.x;
	this->y = source.y;
}

uint16_t Tile::getInfo() {return tileInfo;}

inline void Tile::setBit(mapBit setBit, bool toState)
{
	uint16_t readBit = (0b1 << (uint16_t)setBit); //Shift left to the bit which should be read

    if (toState) {
        tileInfo = tileInfo | (readBit); //Sets bit to true
    } else {
        tileInfo = tileInfo & ~readBit; //Sets bit to false
    }
}

inline bool Tile::getBit(mapBit get)
{
	return (tileInfo >> (uint16_t)get) & 0b1;
}

int Tile::wallCount()
{
	int walls = 0;
	for (int i = 0; i < 4; i++) {
		if (getBit((mapBit)i)) walls++;
	}
	return walls;
}