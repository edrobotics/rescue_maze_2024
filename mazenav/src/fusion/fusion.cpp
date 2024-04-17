#include <fusion/fusion.h>


TeensyCommunicator tComm {1, 0x69};

MotorControllers motors {&tComm};
Sensors sensors {&tComm};
PoseEstimator poseEstimator {&sensors};


void fusion::main(communication::Communicator* globComm, PiAbstractor* piAbs)
{
    std::cout << "Init tComm... ";
    tComm.initiate();
    std::cout << "done." << "\n";

    // Set the correct fusion group
    poseEstimator.setFusionGroup(PoseEstimator::fg_simple);

    // Create hardware communication thread. Will run indefinetely
    // std::cout << "Spawn hardwarecommunicator... ";
    std::thread hardwareCommunicator(&TeensyCommunicator::runLoopLooper, &tComm);
    // std::cout << "done." << "\n";
    // std::cout << "Spawn motorDriver... ";
    std::thread motorDriver(motorDriveLoopLooper, globComm, &motors);
    // std::cout << "done." << "\n";
    std::cout << "Spawn poseEstimator... ";
    std::thread poseEst(&PoseEstimator::runLoopLooper, &poseEstimator, globComm);
    std::cout << "done." << "\n";
    
    // while (true)
    // {
        // std::cout << globComm->poseComm.robotFrame << "\n";
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    poseEst.join();
    std::cout << "joined\n";
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