#include <localNav/localNav.h>

void testTransforms();

void localNav::main(communication::Communicator* globComm)
{
    testTransforms();
}


void testTransforms()
{
    // Create initial frames

    Transform tf1 {};
    CoordinateFrame world {nullptr, tf1};
    Transform tf2 {900, 900, 0, 0, 0, 0};
    CoordinateFrame localTile {&world, tf2};
    Transform roboTrans {150, 150, 0, 0, 0, 0};
    CoordinateFrame robot {&localTile, roboTrans};
    
    Transform globalRobot {robot.transformTo(world)};

    // // Print frames
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";
    std::cout << "\n" << "\n" << "\n";

    // Move the local tile one step right and rotate 90deg clockwise
    Transform moveAndRotate {300, 0, 0, 0, 0, -90};
    localTile.incrementTransfrom(moveAndRotate);

    globalRobot = robot.transformTo(world);

    // Print robot global frame and check if it is correct
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";
}