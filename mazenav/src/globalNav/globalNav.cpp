#include "globalNav/globalNav.h"

namespace globalNav
{
	void main(communication::Communicator* communicatorInstance)
	{
		try
		{
			MazeNavigator mazeNavigator(communicatorInstance);

			while (true)
			{
				mazeNavigator.makeNavigationDecision();
				mazeNavigator.updateInfoAfterDriving();
			}
		}
		catch(const std::exception& e)
		{
			//Left wall follower
			std::cerr << e.what() << '\n';
		}
		
	}
}