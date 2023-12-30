#include <globalNav/globalNav.h>

using namespace globalNav;

namespace globalNav
{

void main()
{

}

class tile
{
    public:
    uint8_t x;
	uint8_t y;

	tile(uint8_t x, uint8_t y)
	{
		this->x = x;
		this->y = y;
	}

	tile(tile* t)
	{
		this->copy(t);
	}

	uint16_t g;
	uint16_t h;

	int cost() const { return g + h; }

	tile* parent;
	
	void copy(tile source)
	{
		this->info = source.getInfo();
		this->x = source.x;
		this->y = source.y;
	}

	uint16_t getInfo() {return info;}

	void setBit(mBit set, bool state)
	{
		uint16_t bit = (0b1 << (uint16_t)set);

        if (state) {
            info = info | (bit);
        } else {
            info = info & ~bit;
        }
	}

	bool getBit(mBit get)
	{
		return ((info >> (uint16_t)get) & 0b1) == 1;
	}

	int wallCount()
	{
		int walls = 0;
		for (int i = 0; i < 4; i++) {
			if (getBit((mBit)i)) walls++;
		}
		return walls;
	}
	
	private:
	uint16_t info;
};
}