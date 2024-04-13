#include "localNav/localNav.h"

// void testDriving(KinematicDriver* driver);
void testTfsys();




void localNav::main(communication::Communicator* globComm)
{
    PathFollower pathFollower {globComm};

    globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
    globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
    globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
    // globComm->navigationComm.pushCommand(communication::DriveCommand::driveForward);
    // globComm->navigationComm.pushCommand(communication::DriveCommand::turnLeft);
    
    std::thread pFollow(&PathFollower::runLoopLooper, &pathFollower);
    
    pFollow.join();

    // testTfsys();
    // KinematicDriver driver {globComm};
    // testDriving(&driver);
}

// void testDriving(KinematicDriver* driver)
// {
//     while(true)
//     {
//         driver->testComm();
//     }
// }

void testTfsys()
{

    // Transform::test();

    // Create initial frames

    Transform tf1 {};
    CoordinateFrame world {nullptr, tf1};
    Transform tf2 {900, 900, 0, 0, 0, 0};
    CoordinateFrame localTile {&world, tf2};
    Transform roboTrans {150, 150, 0, 0, 0, -0.69};
    CoordinateFrame robot {&localTile, roboTrans};
    
    Transform globalRobot {robot.getTransformRootTo(&world)};

    // // Print frames
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";
    std::cout << "\n" << "\n" << "\n";

    // Move the local tile one step right and rotate 90deg clockwise
    Transform moveAndRotate {398, 0, 0, 0, 0, -2.11};
    Transform forward {0, 300, 0, 0, 0, 0};
    // localTile.incrementTransfrom(moveAndRotate);
    localTile.ghostMove(moveAndRotate);

    globalRobot = robot.getTransformRootTo(&world);

    // Print robot global frame and check if it is correct
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";

    // localTile.printChildNum();
    // std::vector<CoordinateFrame*> frames;
    // for (int i=0;i<10;++i)
    // {
    //     frames.push_back(new CoordinateFrame{&localTile, Transform{}});
    //     localTile.printChildNum();
    // }
    // localTile.printChildNum();
    // localTile.deleteChildren();
    // localTile.printChildNum();
    std::cout << "DONE" << "\n";

}