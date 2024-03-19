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

    if (getIsTofXAbsolute())
    {
        resultPose.robotFrame.transform.rot_z = getTofZRot();
        resultPose.robotFrame.transform.pos_x = getTofXTrans(resultPose.robotFrame.transform.rot_z);
    }
    else
    {
        resultPose.robotFrame.transform.rot_z = globalPose.robotFrame.transform.rot_z + getIMURotDiff();
        if (getIsTofDiff())
        {
            resultPose.robotFrame.transform.pos_x = globalPose.robotFrame.transform.pos_x + ( (getWheelTransDiff()+getTofTransYDiff())/2 ) * cos(resultPose.robotFrame.transform.rot_z);
        }
        else
        {
            // What to do here? I cannot update the pose
            #warning unhandled
        }
    }

    if (getIsTofYAbsolute())
    {
        resultPose.robotFrame.transform.pos_y = getTofYTrans(resultPose.robotFrame.transform.rot_z, TOF_FY_OFFSET, TOF_FX_OFFSET);
    }
    else
    {
        if (getIsTofDiff())
        {
            resultPose.robotFrame.transform.pos_y = globalPose.robotFrame.transform.pos_y + ( (getWheelTransDiff()+getTofTransYDiff())/2 ) * sin(resultPose.robotFrame.transform.rot_z);
        }
        else
        {
            // What to do here? I cannot update the pose
            #warning unhandled
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

    wrapPoseComm(resultPose);

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateLidarSimple()
{
    communication::PoseCommunicator resultPose {};

    std::cout << "[PoseEstimator][ERROR]: Lidar+simple pose estimation not implemented";

    wrapPoseComm(resultPose);

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateIMU()
{
    communication::PoseCommunicator resultPose {};

    wrapPoseComm(resultPose);

    return resultPose;
}


double PoseEstimator::wrapValue(double value, double min, double max)
{
    double diffCorr {max-min};
    while (value>max)
    {
        value-=diffCorr;
    }
    while (value<=min)
    {
        value+=diffCorr;
    }

    return value;
}


void PoseEstimator::wrapPoseComm(communication::PoseCommunicator& poseComm)
{
    Transform ghostTf {};
    // Z
    if (poseComm.robotFrame.transform.rot_z < minZRot)
    {
        ghostTf.pos_x += GRID_SIZE;
        ghostTf.rot_z += M_PI_2;
    }
    else if (poseComm.robotFrame.transform.rot_z >= maxZRot)
    {
        ghostTf.pos_y += GRID_SIZE;
        ghostTf.rot_z += -M_PI_2;
    }


    if (poseComm.robotFrame.transform.pos_x < minXPos)
    {
        ghostTf.pos_x += -GRID_SIZE;
    }
    else if (poseComm.robotFrame.transform.pos_x >= maxXPos)
    {
        ghostTf.pos_x += GRID_SIZE;
    }


    if (poseComm.robotFrame.transform.pos_y < minYPos)
    {
        ghostTf.pos_y += -GRID_SIZE;
    }
    else if (poseComm.robotFrame.transform.pos_y >= maxYPos)
    {
        ghostTf.pos_y += GRID_SIZE;
    }

    poseComm.localTileFrame.ghostMove(ghostTf);
}


double PoseEstimator::getWheelTransDiff()
{
    MotorControllers::Distances motorDistances {sensors.motors.motorDistances};
    double average = (motorDistances.lb + motorDistances.lf + motorDistances.rf + motorDistances.rb)/4.0;
    double diff = average-lastWheelTransDiffDist;
    lastWheelTransDiffDist = average;
    return diff;
}


double PoseEstimator::getTofTransYDiff()
{
    Tof::TofData td {sensors.tofs.tofData};
    double lfDiff {td.lf-lastTofLF};
    double lbDiff {td.lf-lastTofLB};
    double bDiff {td.lf-lastTofB};

    double avgDiff {(lfDiff+lbDiff+bDiff)/3.0};

    lastTofLF = td.lf;
    lastTofLB = td.lb;
    lastTofB = td.b;

    return avgDiff;
    
}


double PoseEstimator::getTofYTrans(double angle, double yoffset, double xoffset)
{
    Tof::TofData td {sensors.tofs.tofData};
    // Get Y distances
    double lfY {td.lf*cos(angle)};
    double rfY {td.rf*cos(angle)};
    double bY {td.b*cos(angle)};

    // Change to robot centre
    lfY = lfY+yoffset*cos(angle)+xoffset*sin(angle);
    rfY = rfY+yoffset*cos(angle)+xoffset*sin(angle);
    bY = bY+yoffset*cos(angle);

    // Wrap values to local tile
    lfY = wrapValue(lfY, 0, GRID_SIZE-1);
    rfY = wrapValue(rfY, 0, GRID_SIZE-1);
    bY = wrapValue(bY, 0, GRID_SIZE-1);

    // Average
    double average {(lfY+rfY+bY)/3.0};

    return average;
}


double PoseEstimator::getTofXTrans(double angle)
{
    Tof::TofData td {sensors.tofs.tofData};
    double x1 {};
    double x2 {};
    double resultX {};

    if (getIsTofXLeft())
    {
        x1 = getCentredistanceFromTwoTof(td.lf, td.lb, TOF_SX_OFFSET, angle);
        if (getIsTofXRight())
        {
            x2 = GRID_SIZE - (-getCentredistanceFromTwoTof(td.rf, td.rb, TOF_SX_OFFSET, angle));
            resultX = (x1+x2)/2.0;
        }
        else
        {
            resultX = x1;
        }
    }
    else if (getIsTofXRight())
    {
        resultX = GRID_SIZE - (-getCentredistanceFromTwoTof(td.rf, td.rb, TOF_SX_OFFSET, angle));
    }
    else
    {
        // What to do here? Not enough information, should never be here
        #warning unhandled: cannot compute XTrans with no walls
    }

    return resultX;
}

double PoseEstimator::getTofZRot()
{
    Tof::TofData td {sensors.tofs.tofData};
    double angle1 {};
    double angle2 {};
    double resultAngle {};
    if (getIsTofXLeft())
    {
        angle1 = getAngleFromTwoTof(td.lf, td.lb, TOF_SY_OFFSET*2);
        if (getIsTofXRight())
        {
            angle2 = -getAngleFromTwoTof(td.rf, td.rb, TOF_SY_OFFSET*2);
            resultAngle = (angle1+angle2)/2.0;
        }
        else
        {
            resultAngle = angle1;
        }
    }
    else if (getIsTofXRight())
    {
        resultAngle = -getAngleFromTwoTof(td.rf, td.rb, TOF_SY_OFFSET*2);
    }
    else
    {
        // What to do here? Not enough information, should never be here
        #warning unhandled: cannot compute rotation with no walls
    }

    return resultAngle;
}

double PoseEstimator::getAngleFromTwoTof(double d1, double d2, double yoffset)
{
    return atan((d1-d2)/yoffset);
}

double PoseEstimator::getCentredistanceFromTwoTof(double d1, double d2, double centreoffset, double angle)
{
    return ((d1+d2)/2+centreoffset)*cos(angle);
}



bool PoseEstimator::getIsTofXAbsolute()
{

}

bool PoseEstimator::getIsTofXLeft()
{

}

bool PoseEstimator::getIsTofXRight()
{

}

bool PoseEstimator::getIsTofYAbsolute()
{

}

bool PoseEstimator::getIsTofDiff()
{

}
