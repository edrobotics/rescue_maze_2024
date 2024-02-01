#include <localNav/localNav.h>

void testDriving(KinematicDriver* driver);
void testTransforms();

void localNav::main(communication::Communicator* globComm)
{
    // testTransforms();
    KinematicDriver driver {globComm};
    testDriving(&driver);
}

void testDriving(KinematicDriver* driver)
{
    while(true)
    {
        driver->testComm();
    }
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
    
    Transform globalRobot {robot.getTransformTo(&world)};

    // // Print frames
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";
    std::cout << "\n" << "\n" << "\n";

    // Move the local tile one step right and rotate 90deg clockwise
    Transform moveAndRotate {0, 300, 0, 0, 0, -M_PI_2};
    localTile.incrementTransfrom(moveAndRotate);

    globalRobot = robot.getTransformTo(&world);

    // Print robot global frame and check if it is correct
    std::cout << "localTile:   " << localTile << "\n";
    std::cout << "localRobot:  " << robot << "\n";
    std::cout << "globalRobot: " << globalRobot << "\n";
}