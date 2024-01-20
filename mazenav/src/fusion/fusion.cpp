#include <fusion/fusion.h>


TeensyCommunicator communicator {1, 0x69};

MotorControllers motors {&communicator};
Sensors sensors {&communicator};

void fusion::main()
{
    communicator.initiate();

    // Create hardware communication thread. Will run indefinetely
    std::thread hardwareCommunicator(&TeensyCommunicator::runLoopLooper, &communicator);
    
    while (true)
    {
        // Update the sensors
        sensors.update();

        // Update the pose (To be done)
    }
    
}