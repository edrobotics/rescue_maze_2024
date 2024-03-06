#include "fusion/PoseEstimator.h"

PoseEstimator::PoseEstimator(communication::Communicator* globalCommunicator, TeensyCommunicator* teensyCommunicator)
    : globComm {globalCommunicator},
      tComm {teensyCommunicator},
      sensors {tComm}
{
    #warning unsure what to do here. Initialize sensors?

}


void PoseEstimator::setFusionGroup(FusionGroup fgroup)
{
    fusionGroup = fgroup;
}

void PoseEstimator::begin()
{
    stopThread = false;
    #warning usure whether this is OK or not
    updater = std::thread{&PoseEstimator::runLoopLooper, this};
}

void PoseEstimator::stop()
{
    stopThread = true;
    updater.join();
    stopThread = false;
}

void PoseEstimator::runLoopLooper()
{
    while (!stopThread)
    {
        runLoop();
    }
    return;
}

void PoseEstimator::runLoop()
{
    // Currently: update as fast as possible for the given FusionGroup
    update(fusionGroup);
}

void PoseEstimator::update(FusionGroup fgroup)
{
    #warning check synchronisation (does it happen and do we want it?)
    sensors.update();

    communication::PoseCommunicator poseResult {globComm->poseComm};
    switch (fgroup)
    {
        case fg_lidar:
            poseResult = updateLidar();
            break;
        case fg_simple:
            poseResult = updateSimple();
            break;
        case fg_imu:
            poseResult = updateIMU();
            break;
        case fg_lidar_imu:
            std::cout << "[PoseEstimator][ERROR]: Lidar+IMU pose estimation not implemented";
            break;
        case fg_lidar_simple:
            poseResult = updateLidarSimple();
            break;
        case fg_none:
            std::cout << "[PoseEstimator][ERROR]: No FusionGroup selected";
            break;
        default:
            std::cout << "[PoseEstimator][ERROR]: Default case reached";
            break;
    }

    // Write the new pose with only changes.
    #warning concurrency issues?
    globComm->poseComm = poseResult;
}






communication::PoseCommunicator PoseEstimator::updateSimple()
{
    communication::PoseCommunicator resultPose {};

    // This is where the magic happens

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateLidar()
{
    communication::PoseCommunicator resultPose {};

    std::cout << "[PoseEstimator][ERROR]: Lidar pose estimation not implemented";

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateLidarSimple()
{
    communication::PoseCommunicator resultPose {};

    std::cout << "[PoseEstimator][ERROR]: Lidar+simple pose estimation not implemented";

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateIMU()
{
    communication::PoseCommunicator resultPose {};

    return resultPose;
}
