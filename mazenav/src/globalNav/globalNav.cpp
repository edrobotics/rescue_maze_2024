#include "globalNav/globalNav.h"

using namespace communication;

namespace globalNav
{
	void main(Communicator* communicatorInstance)
	{
		MazeNavigator mazeNavigator(communicatorInstance);

		while (true)
		{
			mazeNavigator.makeDecision();
		}
	}
}