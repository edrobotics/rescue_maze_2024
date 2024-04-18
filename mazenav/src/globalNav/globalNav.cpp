#include "globalNav/globalNav.h"

namespace globalNav
{
	void main(communication::Communicator* communicatorInstance)
	{
		MazeNavigator mazeNavigator(communicatorInstance);

		try{
			mazeNavigator.init();

			while (true){
				mazeNavigator.makeNavigationDecision();
				mazeNavigator.updateInfoAfterDriving();
			}
		}
		catch(const std::exception& e){
			//Left wall follower
			std::cerr << e.what() << '\n';
			while (true)
			{
				try
				{
					mazeNavigator.followLeftWall();
				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
			}
			
		}
		
	}
}