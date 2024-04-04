#include <fusion/fusion.h>


TeensyCommunicator tComm {1, 0x69};

MotorControllers motors {&tComm};
Sensors sensors {&tComm};




void fusion::main(communication::Communicator* globComm)
{
    tComm.initiate();
    PoseEstimator poseEstimator {globComm, &tComm, &sensors};

    // Create hardware communication thread. Will run indefinetely
    std::thread hardwareCommunicator(&TeensyCommunicator::runLoopLooper, &tComm);
    std::thread motorDriver(motorDriveLoopLooper, globComm, &motors);
    poseEstimator.begin();
    
    while (true)
    {
        std::cout << globComm->poseComm.robotFrame << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    poseEstimator.stop();
    motorDriver.join();
    hardwareCommunicator.join();
    
}

void fusion::motorDriveLoop(communication::Communicator* gCom, MotorControllers* mot)
{
    // Set speeds from gCom to motor control object
    mot->setSpeeds(gCom->motors.getSpeeds());
    // Set values from motor control object to teensy communicator
    mot->setVals();
    // Sleep to not occupy the mutexes
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void fusion::motorDriveLoopLooper(communication::Communicator* gCom, MotorControllers* mot)
{
    while(true)
    {
        motorDriveLoop(gCom, mot);
    }
}