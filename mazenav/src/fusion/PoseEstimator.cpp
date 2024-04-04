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
    #warning unsure whether this is OK or not
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
    communication::PoseCommunicator lastPose {globalPose};

    // This is where the magic happens

    // Update wheel positions
    calcWheelDistanceDiffs();

    // Rotation:
    Average rotAbs {};
    ConditionalAverageTerm robotAngle {getTofZRot(lastPose.robotFrame.transform.rot_z)};
    rotAbs.terms.push_back(robotAngle);

    ConditionalAverageTerm wheelRot {getWheelRotDiff()};
    wheelRot.value += lastPose.robotFrame.transform.rot_z;
    rotAbs.terms.push_back(wheelRot);

    ConditionalAverageTerm imuRot {getIMURotDiff()};
    imuRot.value += lastPose.robotFrame.transform.rot_z;
    rotAbs.terms.push_back(imuRot);

    for (auto& term : rotAbs.terms)
    {
        term.value = wrapValue(term.value, minZRot, maxZRot);
    }

    double ang {};
    try
    {
        resultPose.robotFrame.transform.rot_z = rotAbs.calc();
        ang = robotAngle.value;
    }
    catch(const std::runtime_error& e)
    {
        std::cerr << e.what() << " : " << "Cannot compute robot angle" << '\n';
        #warning what to do here, with no angle? Also what to do for further angle requirements
    }
    
    
    // Translation, abs:
    Average transXAbs {};
    Average transYAbs {};

    transXAbs.terms.push_back(getTofXTrans(ang));
    transYAbs.terms.push_back(getTofYTrans(ang, TOF_FY_OFFSET, TOF_FX_OFFSET));

    ConditionalAverageTerm wheelTransX {getWheelTransDiff()};
    ConditionalAverageTerm wheelTransY {wheelTransX};
    wheelTransX.value = wheelTransX.value*cos(resultPose.robotFrame.transform.rot_z) + lastPose.robotFrame.transform.pos_x;
    wheelTransY.value = wheelTransY.value*sin(resultPose.robotFrame.transform.rot_z) + lastPose.robotFrame.transform.pos_y;
    transXAbs.terms.push_back(wheelTransX);
    transYAbs.terms.push_back(wheelTransY);

    ConditionalAverageTerm tofTransX {getTofTransYDiff()};
    ConditionalAverageTerm tofTransY {tofTransX};
    tofTransX.value = tofTransX.value*cos(resultPose.robotFrame.transform.rot_z) + lastPose.robotFrame.transform.pos_x;
    tofTransY.value = tofTransY.value*sin(resultPose.robotFrame.transform.rot_z) + lastPose.robotFrame.transform.pos_y;
    transXAbs.terms.push_back(tofTransX);
    transYAbs.terms.push_back(tofTransY);

    for (auto& term : transXAbs.terms)
    {
        term.value = wrapValue(term.value, minXPos, maxXPos);
    }
    for (auto& term : transYAbs.terms)
    {
        term.value = wrapValue(term.value, minYPos, maxYPos);
    }

    try
    {
        resultPose.robotFrame.transform.pos_x = transXAbs.calc();
    }
    catch(std::runtime_error& e)
    {
        std::cerr << e.what() << " : " << "Cannot compute robot X position" << '\n';
    }

    try
    {
        resultPose.robotFrame.transform.pos_y = transYAbs.calc();
    }
    catch(std::runtime_error& e)
    {
        std::cerr << e.what() << " : " << "Cannot compute robot Y position" << '\n';
    }


    updatePoseComm(resultPose, lastPose);

    return resultPose;

    #warning IMPORTANT: tile changes are broken. Some systems fix it themselves before getting the values here, others are dependent on the wrappose. All need to be the same for averaging.
}

communication::PoseCommunicator PoseEstimator::updateLidar()
{
    communication::PoseCommunicator resultPose {};
    communication::PoseCommunicator lastPose {globComm->poseComm};

    std::cout << "[PoseEstimator][ERROR]: Lidar pose estimation not implemented";

    updatePoseComm(resultPose, lastPose);

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateLidarSimple()
{
    communication::PoseCommunicator resultPose {};
    communication::PoseCommunicator lastPose {globComm->poseComm};

    std::cout << "[PoseEstimator][ERROR]: Lidar+simple pose estimation not implemented";

    updatePoseComm(resultPose, lastPose);

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateIMU()
{
    communication::PoseCommunicator resultPose {};
    communication::PoseCommunicator lastPose {globComm->poseComm};

    updatePoseComm(resultPose, lastPose);

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


// void PoseEstimator::wrapPoseComm(communication::PoseCommunicator& poseComm)
// {
//     Transform ghostTf {};
//     // Z
//     if (poseComm.robotFrame.transform.rot_z < minZRot)
//     {
//         ghostTf.pos_x += GRID_SIZE;
//         ghostTf.rot_z += M_PI_2;
//     }
//     else if (poseComm.robotFrame.transform.rot_z >= maxZRot)
//     {
//         ghostTf.pos_y += GRID_SIZE;
//         ghostTf.rot_z += -M_PI_2;
//     }


//     if (poseComm.robotFrame.transform.pos_x < minXPos)
//     {
//         ghostTf.pos_x += -GRID_SIZE;
//     }
//     else if (poseComm.robotFrame.transform.pos_x >= maxXPos)
//     {
//         ghostTf.pos_x += GRID_SIZE;
//     }


//     if (poseComm.robotFrame.transform.pos_y < minYPos)
//     {
//         ghostTf.pos_y += -GRID_SIZE;
//     }
//     else if (poseComm.robotFrame.transform.pos_y >= maxYPos)
//     {
//         ghostTf.pos_y += GRID_SIZE;
//     }

//     poseComm.localTileFrame.ghostMove(ghostTf);
// }


void PoseEstimator::updatePoseComm(communication::PoseCommunicator& poseComm, communication::PoseCommunicator lastPoseComm)
{
    Transform ghostTf {};

    double rotDiff {poseComm.robotFrame.transform.rot_z - lastPoseComm.robotFrame.transform.rot_z};
    // Turn right
    if ( rotDiff > tileRotDiffThreshold)
    {
        ghostTf.rot_z += -M_PI_2;
        ghostTf.pos_y += GRID_SIZE;
    }
    // Turn left
    else if (rotDiff < -tileRotDiffThreshold)
    {
        ghostTf.rot_z += M_PI_2;
        ghostTf.pos_x += GRID_SIZE;

    }


    double xDiff {poseComm.robotFrame.transform.pos_x - lastPoseComm.robotFrame.transform.pos_x};

    // Move left
    if (xDiff > tileTransXDiffThreshold)
    {
        ghostTf.pos_x += -GRID_SIZE;
    }
    // Move right
    else if (xDiff < -tileTransXDiffThreshold)
    {
        ghostTf.pos_x += GRID_SIZE;
    }


    double yDiff {poseComm.robotFrame.transform.pos_y - lastPoseComm.robotFrame.transform.pos_y};

    // Move back
    if (yDiff > tileTransYDiffThreshold)
    {
        ghostTf.pos_y += -GRID_SIZE;
    }
    // Move forward
    else if (yDiff < -tileTransYDiffThreshold)
    {
        ghostTf.pos_y += GRID_SIZE;
    }


    // Carry out the ghostmove
    poseComm.localTileFrame.ghostMove(ghostTf);
}



void PoseEstimator::calcWheelDistanceDiffs()
{
    MotorControllers::Distances motorDistances {};
    motorDistanceDiffs.lf = motorDistances.lf - lastMotorDistances.lf;
    motorDistanceDiffs.lb = motorDistances.lb - lastMotorDistances.lb;
    motorDistanceDiffs.rf = motorDistances.rf - lastMotorDistances.rf;
    motorDistanceDiffs.rb = motorDistances.rb - lastMotorDistances.rb;

    lastMotorDistances = motorDistances;
}

ConditionalAverageTerm PoseEstimator::getWheelTransDiff()
{
    ConditionalAverageTerm result {0, 0};

    Average avg {};

    avg.terms.push_back(ConditionalAverageTerm{motorDistanceDiffs.lf, 1});
    avg.terms.push_back(ConditionalAverageTerm{motorDistanceDiffs.lb, 1});
    avg.terms.push_back(ConditionalAverageTerm{motorDistanceDiffs.rf, 1});
    avg.terms.push_back(ConditionalAverageTerm{motorDistanceDiffs.rb, 1});

    // Filter out bad values
    for (auto& term : avg.terms)
    {
        if (abs(term.value) > MAX_WHEEL_ODOM_DIFF)
        {
            term.weight = 0;
        }
        else
        {
            term.weight = 1;
        }
    }
    
    try
    {
        result.value = avg.calc();
        result.weight = 1;
    }
    catch (std::runtime_error& e)
    {
        // No usable data, so do not use
        result.weight = 0;
    }

    return result;
}


ConditionalAverageTerm PoseEstimator::getTofTransYDiff()
{
    ConditionalAverageTerm result {0, 0};

    Tof::TofData td {sensors.tofs.tofData};
    ConditionalAverageTerm flDiff {td.fl-lastTofFL, 1};
    ConditionalAverageTerm frDiff {td.fr-lastTofFR, 1};
    ConditionalAverageTerm bDiff {td.b-lastTofB, 1};

    lastTofFL = td.fl;
    lastTofFR = td.fr;
    lastTofB = td.b;

    // If angle is too large the sensors see the side walls instead of the one in front
    #warning this changes with the distance to the wall in front. When we are closer, do we want to accept a greater angle? But how do we know if we are closer?
    if (abs(globComm->poseComm.robotFrame.transform.rot_z) > MAX_Z_ROTATION_Y_TOF_DIFF)
    {
        return result;
    }

    Average avg {};
    avg.terms.push_back(flDiff);
    avg.terms.push_back(frDiff);
    avg.terms.push_back(bDiff);

    for (auto& term : avg.terms)
    {
        if (abs(term.value)>MAX_TOF_Y_DIFF)
        {
            term.weight = 0;
        }
        else
        {
            term.weight = 1;
        }
    }

    // Check if any values were valid
    try
    {
    result.value = avg.calc();
    result.weight = 1;
    }
    catch (std::runtime_error& e)
    {
        // std::cerr << e.what() << '\n';
        // If not, do not use anything from here
        result.weight = 0;
    }


    return result;
}


ConditionalAverageTerm PoseEstimator::getTofYTrans(double angle, double yoffset, double xoffset)
{
    ConditionalAverageTerm result {0, 0};

    // Check if angle is too great
    if (abs(angle)>MAX_Z_ROTATION_Y_TOF_ABS)
    {
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors.tofs.tofData};
    // Get Y distances
    ConditionalAverageTerm flY {td.fl*cos(angle), 1};
    ConditionalAverageTerm frY {td.fr*cos(angle), 1};
    ConditionalAverageTerm bY {td.b*cos(angle), 1};

    // Change to robot centre
    flY.value = flY.value+yoffset*cos(angle)+xoffset*sin(angle);
    frY.value = frY.value+yoffset*cos(angle)+xoffset*sin(angle);
    bY.value = bY.value+yoffset*cos(angle);

    // Average calculation preparation
    Average avg {};
    avg.terms.push_back(flY);
    avg.terms.push_back(frY);
    avg.terms.push_back(bY);

    for (auto& term : avg.terms)
    {
        // Wrap values to local tile
        term.value = wrapValue(term.value, minYPos, maxYPos);

        // Check if sensor values can be used
        if (term.value > MAX_TOF_Y_DIST_ABS)
        {
            // Reject the term/value
            term.weight = 0;
        }
        else
        {
            // Accept the term/value
            term.weight = 1;
        }
    }

    try
    {
        result.value = avg.calc();
    }
    catch(std::runtime_error& e)
    {
        // std::cerr << e.what() << '\n';
        // If not usable, do not use
        result.weight = 0;
    }
    

    return result;
}


ConditionalAverageTerm PoseEstimator::getTofXTrans(double angle)
{
    ConditionalAverageTerm result {0, 0};

    if (abs(angle)>MAX_ZROT_XTRANS_TOF_ABS)
    {
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors.tofs.tofData};
    double x1 {};
    double x2 {};

    if (getIsTofXLeft())
    {
        x1 = getCentredistanceFromTwoTof(td.fl, td.fr, TOF_SX_OFFSET, angle);
        x1 = wrapValue(x1, minXPos, maxXPos);
        if (getIsTofXRight())
        {
            x2 = GRID_SIZE - (-getCentredistanceFromTwoTof(td.rf, td.rb, TOF_SX_OFFSET, angle));
            x2 = wrapValue(x2, minXPos, maxXPos);
            result.value = (x1+x2)/2.0;
        }
        else
        {
            result.value = x1;
        }

        result.weight = 1;
    }
    else if (getIsTofXRight())
    {
        result.value = GRID_SIZE - (-getCentredistanceFromTwoTof(td.rf, td.rb, TOF_SX_OFFSET, angle));
        result.value = wrapValue(result.value, minXPos, maxXPos);
        result.weight = 1;
    }
    else
    {
        // std::cerr << "Cannot compute ToF X trans without walls" << "\n";
        result.weight = 0;
    }

    return result;
}

ConditionalAverageTerm PoseEstimator::getTofZRot(double curAng)
{
    ConditionalAverageTerm result {0, 0};

    if (abs(curAng)>MAX_ZROT_XTRANS_TOF_ABS)
    {
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors.tofs.tofData};
    double angle1 {};
    double angle2 {};
    if (getIsTofXLeft())
    {
        angle1 = getAngleFromTwoTof(td.fl, td.fr, TOF_SY_OFFSET*2);
        angle1 = wrapValue(angle1, minZRot, maxZRot);
        if (getIsTofXRight())
        {
            angle2 = -getAngleFromTwoTof(td.rf, td.rb, TOF_SY_OFFSET*2);
            angle2 = wrapValue(angle2, minZRot, maxZRot);
            result.value = (angle1+angle2)/2.0;
        }
        else
        {
            result.value = angle1;
        }
        result.weight = 1;
    }
    else if (getIsTofXRight())
    {
        result.value = -getAngleFromTwoTof(td.rf, td.rb, TOF_SY_OFFSET*2);
        result.value = wrapValue(result.value, minZRot, maxZRot);
        result.weight = 1;
    }
    else
    {
        // std::cerr << "Cannot compute ToF Z angle without walls";
        result.weight = 0;
    }

    return result;
}

ConditionalAverageTerm PoseEstimator::getWheelRotDiff()
{
    ConditionalAverageTerm result {0, 0};

    #warning not yet implemented

    return result;
}


ConditionalAverageTerm PoseEstimator::getIMURotDiff()
{
    ConditionalAverageTerm result {0, 0};
    double angle {sensors.imu0.angles.z};
    result.value = angle-lastImuAngle;

    // Prepare for next iteration
    lastImuAngle = angle;

    // Check if angle within span
    #warning assumes that no angle changes > 180 degrees can happen in one iteration
    if (result.value > M_PI)
    {
        result.value = result.value - 2*M_PI;
    }
    // Setting to 1 as default
    result.weight = 1;

    // Check if diff too large
    if (abs(result.value) > MAX_IMU_Z_ROT_DIFF)
    {
        // If data unusable, do not use
        result.weight = 0;
    }

    // Return
    return result;

}



double PoseEstimator::getAngleFromTwoTof(double d1, double d2, double yoffset)
{
    return atan((d1-d2)/yoffset);
}

double PoseEstimator::getCentredistanceFromTwoTof(double d1, double d2, double centreoffset, double angle)
{
    return ((d1+d2)/2+centreoffset)*cos(angle);
}



bool PoseEstimator::getIsTofXLeft()
{
    Tof::TofData td {sensors.tofs.tofData};

    if (td.lf>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    if (td.lb>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    return true;

}

bool PoseEstimator::getIsTofXRight()
{
    Tof::TofData td {sensors.tofs.tofData};

    if (td.rf>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    if (td.rb>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    return true;

}


// uint8_t PoseEstimator::getIsTofYDiff()
// {
//     // If angle is too large the sensors see the side walls instead of the one in front
//     static const double MAX_Z_ROTATION_FOR_Y_DIFF {M_PI_4/2.0};
//     #warning this changes with the distance to the wall in front. When we are closer, do we want to accept a greater angle? But how do we know if we are closer?
//     if (abs(globComm->poseComm.robotFrame.transform.rot_z) > MAX_Z_ROTATION_FOR_Y_DIFF)
//     {
//         return 0;
//     }

//     // Calculate the diffs
//     Tof::TofData td {sensors.tofs.tofData};
//     double diffFL {td.fl-lastTofFL};
//     double diffFR {td.fr-lastTofFR};
//     double diffB {td.b - lastTofB};

//     // Max diff limit
//     static const int MAX_TOF_Y_DIFF {50};
//     // Check if the diffs are OK
//     uint8_t returnInt {};
//     if (abs(diffFL)>MAX_TOF_Y_DIFF)
//     {
//         returnInt |= Y_TRANS_DIFF_FL;
//     }
//     if (abs(diffFR)>MAX_TOF_Y_DIFF)
//     {
//         returnInt |= Y_TRANS_DIFF_FR;
//     }
//     if (abs(diffB)>MAX_TOF_Y_DIFF)
//     {
//         returnInt |= Y_TRANS_DIFF_B;
//     }

//     return returnInt;
// }
