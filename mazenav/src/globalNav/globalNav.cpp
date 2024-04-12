#include "globalNav/globalNav.h"

namespace globalNav
{
	void main(communication::Communicator* communicatorInstance)
	{
		MazeNavigator mazeNavigator(communicatorInstance);

		while (true)
		{
			mazeNavigator.makeNavigationDecision();
			// #error I need to update the map, check for victims and so on
		}
	}
}