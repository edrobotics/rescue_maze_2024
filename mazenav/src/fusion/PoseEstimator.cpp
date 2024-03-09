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





#warning currently a lot of code without the right conditional logic. Do not expect to work at all
communication::PoseCommunicator PoseEstimator::updateSimple()
{
    communication::PoseCommunicator resultPose {};
    communication::PoseCommunicator globalPose {globComm->poseComm};

    // This is where the magic happens

    // Check if you can use the absolute values:
    #warning todo: chekc if usable and then only use if usable

    // Start with the absolute that are useable

    // Increments:

    if (getIsTofXAbsolute())
    {
        resultPose.robotFrame.transform.rot_z = getTofZRot();
        resultPose.robotFrame.transform.pos_x = getTofXTrans();
    }
    else
    {
        resultPose.robotFrame.transform.rot_z = globalPose.robotFrame.transform.rot_z + getIMURotDiff();
        if (getIsTofDiff())
        {
            resultPose.robotFrame.transform.pos_x = globalPose.robotFrame.transform.pos_x + ( (getWheelTransDiff()+getTofTransDiff())/2 ) * cos(resultPose.robotFrame.transform.rot_z);
        }
        else
        {
            // What to do here? I cannot update the pose
        }
    }

    if (getIsTofYAbsolute())
    {
        resultPose.robotFrame.transform.pos_y = getTofYTrans();
    }
    else
    {
        if (getIsTofDiff())
        {
            resultPose.robotFrame.transform.pos_y = globalPose.robotFrame.transform.pos_y + ( (getWheelTransDiff()+getTofTransDiff())/2 ) * sin(resultPose.robotFrame.transform.rot_z);
        }
        else
        {
            // What to do here? I cannot update the pose
        }
    }

    // Bounds checking and tile changes
    wrapPoseComm(resultPose);


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


void PoseEstimator::wrapPoseComm(communication::PoseCommunicator& poseComm)
{
    if (poseComm.robotFrame.transform.pos_y > minYPos)
    {
        // Either modulo magic or change the "if" to "while"
    }
    if (poseComm.robotFrame.transform.pos_y < maxYPos)
    {

    }

}