#include "globalNav/globalNav.h"

namespace globalNav
{
	void main(communication::Communicator* communicatorInstance)
	{
		MazeNavigator mazeNavigator(communicatorInstance);

		while (true)
		{
			mazeNavigator.exploreMaze();
		}
	}
}