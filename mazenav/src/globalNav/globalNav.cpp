#include "globalNav/globalNav.h"

namespace globalNav
{
	void main(communication::Communicator* communicatorInstance)
	{
		MazeNavigator mazeNavigator(communicatorInstance);

		while (true)
		{
			try
			{
				mazeNavigator.followLeftWall();
			}
			catch(...)
			{
			}
		}

		// try{
		// 	mazeNavigator.init();

		// 	while (true){
		// 		mazeNavigator.makeNavigationDecision();
		// 		mazeNavigator.updateInfoAfterDriving();
		// 	}
		// }
		// catch(...){
		// 	//Left wall follower
		// 	communicatorInstance->logger.logToAll("globNav - Trying left wal following");
		// 	while (true)
		// 	{
		// 		try
		// 		{
		// 			mazeNavigator.followLeftWall();
		// 		}
		// 		catch(...)
		// 		{
		// 		}
		// 	}
			
		// }
		
	}
}