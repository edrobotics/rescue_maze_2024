#include <globalNav/globalNav.h>

using namespace communication;
using communication::driveCommand;

namespace globalNav
{
	int leftIterator = 0;

	struct wallInfo
	{
		bool front;
		bool left;
		bool back;
		bool right;
	};

	wallInfo getInfo()
	{

	}

	driveCommand makeDecision(wallInfo wInfo)
	{
		if (!wInfo.left && leftIterator < 2)
		{
			leftIterator++;
			return turnLeft;
		}
		leftIterator = 0;
		if (!wInfo.front) return driveForward;
		return turnRight;
	}

	void main(Communicator* communicator)
	{
		// mxWalls = mutexWalls;
		// mxComm = mutexComm;
		while (true)
		{
			communicator->navigationComm.pushCommand(makeDecision(getInfo()));
		}
	}

	tile::tile(uint8_t x, uint8_t y)
	{
		this->x = x;
		this->y = y;
	}

	tile::tile(tile* t)
	{
		this->copy(t);
	}
	
	void tile::copy(tile source)
	{
		this->info = source.getInfo();
		this->x = source.x;
		this->y = source.y;
	}

	uint16_t tile::getInfo() {return info;}

	void tile::setBit(mBit set, bool state)
	{
		uint16_t bit = (0b1 << (uint16_t)set);

        if (state) {
            info = info | (bit);
        } else {
            info = info & ~bit;
        }
	}

	bool tile::getBit(mBit get)
	{
		return ((info >> (uint16_t)get) & 0b1) == 1;
	}

	int tile::wallCount()
	{
		int walls = 0;
		for (int i = 0; i < 4; i++) {
			if (getBit((mBit)i)) walls++;
		}
		return walls;
	}
}