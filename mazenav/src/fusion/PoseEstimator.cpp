#include "fusion/PoseEstimator.h"

PoseEstimator::PoseEstimator(Sensors* sens)
    // : globComm {globalCommunicator},
    //   : tComm {teensyCommunicator},
    : sensors {sens}
{
}


void PoseEstimator::setFusionGroup(FusionGroup fgroup)
{
    fusionGroup = fgroup;
}

// void PoseEstimator::begin()
// {
//     stopThread = false;
//     #warning unsure whether this is OK or not
//     updater = std::thread{&PoseEstimator::runLoopLooper, this};
// }

// void PoseEstimator::stop()
// {
//     stopThread = true;
//     updater.join();
//     stopThread = false;
// }

void PoseEstimator::runLoopLooper(communication::Communicator* globComm)
{
    // std::cout << "Started runLoopLooper" << "\n";
    this->globComm = globComm;
    // std::cout << "Stored globcomm: " << this->globComm << "\n";
    while (true)
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
    // std::cout << "In update" << std::endl;
    #warning 
    sensors->update(true);
    // Should copy the tree of poseComm into poseResult
    communication::PoseCommunicator poseResult {};
    // Lock poseComm to prevent concurrency issues with the targetPoint
    globComm->poseComm.mtx_general.lock();
    // Save the current value so that we can set it as the old one later
    // CoordinateFrame lastRobot {globComm->poseComm.robotFrame.getWithoutChildren()};
    // std::cout << "lastRobot: " << lastRobot << "\n";
    switch (fgroup)
    {
        case fg_lidar:
            poseResult = updateLidar();
            break;
        case fg_simple:
            // std::cout << "Reached simple" << std::endl;
            poseResult = updateSimple();
            // std::cout << "Finished simple" << std::endl;
            break;
        case fg_imu:
            poseResult = updateIMU();
            break;
        case fg_lidar_imu:
            std::cerr << "[PoseEstimator][ERROR]: Lidar+IMU pose estimation not implemented";
            break;
        case fg_lidar_simple:
            poseResult = updateLidarSimple();
            break;
        case fg_none:
            std::cerr << "[PoseEstimator][ERROR]: No FusionGroup selected";
            break;
        default:
            std::cerr << "[PoseEstimator][ERROR]: Default case reached";
            break;
    }

    // Set fresh to false to startup tile change checking.
    if (poseResult.freshness>0)
    {
        --(poseResult.freshness);
    }

    // Set the lastRobot with the value stored at the top
    // poseResult.lastRobotFrame = lastRobot.getWithoutChildren();
    // poseResult.lastRobotFrame.setParentTS(&(poseResult.localTileFrame));

    // Handle updating of localTile (global coordinates)
    updatePoseComm(poseResult);

    // Write the new pose with only changes.
    #warning concurrency issues?
    globComm->poseComm = poseResult;
    // Unlock poseComm to allow access to targetPoint again.
    globComm->poseComm.mtx_general.unlock();

    std::cout << "robotFrame: " << globComm->poseComm.robotFrame << /*"  lastRobotFrame: " << globComm->poseComm.lastRobotFrame << */ "\n";
}





#warning currently a lot of code without the right conditional logic. Do not expect to work at all
communication::PoseCommunicator PoseEstimator::updateSimple()
{
    // communication::PoseCommunicator lastPose {globComm->poseComm};
    communication::PoseCommunicator resultPose {globComm->poseComm};
    resultPose.lastRobotFrame = resultPose.robotFrame;
    // Update the times
    resultPose.lastRobotTime = resultPose.curRobotTime;
    resultPose.curRobotTime = std::chrono::steady_clock::now();
    // CoordinateFrame resultRobot {globComm->poseComm.robotFrame};

    // This is where the magic happens

    // Update wheel positions
    calcWheelDistanceDiffs();

    // Rotation:
    Average rotAbs {};
    ConditionalAverageTerm robotAngle {getTofZRot(resultPose.lastRobotFrame.transform.rot_z)};
    rotAbs.terms.push_back(robotAngle);

    ConditionalAverageTerm wheelRot {getWheelRotDiff()};
    wheelRot.value += resultPose.lastRobotFrame.transform.rot_z;
    rotAbs.terms.push_back(wheelRot);

    ConditionalAverageTerm imuRot {getIMURotDiff()};
    // std::cout << imuRot.value << "\n";
    imuRot.value += resultPose.lastRobotFrame.transform.rot_z;
    rotAbs.terms.push_back(imuRot);

    rotAbs.stripZeroWeight();
    wrapVectorSameScale(rotAbs.terms, minZRot, maxZRot);

    // double ang {};
    try
    {
        resultPose.robotFrame.transform.rot_z = wrapValue(rotAbs.calc(), minZRot, maxZRot);
        // ang = robotAngle.value;
    }
    catch(const std::runtime_error& e)
    {
        std::cerr << e.what() << " : " << "Cannot compute robot angle" << '\n';
        #warning what to do here, with no angle? Also what to do for further angle requirements
    }
    
    
    // Translation, abs:
    Average transXAbs {};
    Average transYAbs {};

    transXAbs.terms.push_back(getTofXTrans(resultPose.robotFrame.transform.rot_z));
    transYAbs.terms.push_back(getTofYTrans(resultPose.robotFrame.transform.rot_z, TOF_FY_OFFSET, TOF_FX_OFFSET));

    // ConditionalAverageTerm wheelTransX {getWheelTransDiff()};
    // ConditionalAverageTerm wheelTransY {wheelTransX};
    // wheelTransX.value = wheelTransX.value*cos(resultPose.robotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_x;
    // wheelTransY.value = wheelTransY.value*sin(resultPose.robotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_y;
    // transXAbs.terms.push_back(wheelTransX);
    // transYAbs.terms.push_back(wheelTransY);

    // ConditionalAverageTerm tofTransX {getTofTransYDiff()};
    // ConditionalAverageTerm tofTransY {tofTransX};
    // tofTransX.value = tofTransX.value*cos(resultPose.robotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_x;
    // tofTransY.value = tofTransY.value*sin(resultPose.robotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_y;
    // transXAbs.terms.push_back(tofTransX);
    // transYAbs.terms.push_back(tofTransY);

    transXAbs.stripZeroWeight();
    wrapVectorSameScale(transXAbs.terms, minXPos, maxXPos);

    transYAbs.stripZeroWeight();
    wrapVectorSameScale(transYAbs.terms, minYPos, maxYPos);

    try
    {
        resultPose.robotFrame.transform.pos_x = wrapValue(transXAbs.calc(), minXPos, maxXPos);
    }
    catch(std::runtime_error& e)
    {
        // std::cerr << e.what() << " : " << "Cannot compute robot X position" << '\n';
    }

    try
    {
        resultPose.robotFrame.transform.pos_y = wrapValue(transYAbs.calc(), minXPos, maxXPos);
    }
    catch(std::runtime_error& e)
    {
        // std::cerr << e.what() << " : " << "Cannot compute robot Y position" << '\n';
    }

    calcSpeeds(resultPose);

    return resultPose;

    #warning (fixed?) IMPORTANT: tile changes are broken. Some systems fix it themselves before getting the values here, others are dependent on the wrappose. All need to be the same for averaging.
}

communication::PoseCommunicator PoseEstimator::updateLidar()
{
    communication::PoseCommunicator resultPose {globComm->poseComm};

    std::cout << "[PoseEstimator][ERROR]: Lidar pose estimation not implemented";

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateLidarSimple()
{
    communication::PoseCommunicator resultPose {globComm->poseComm};

    std::cout << "[PoseEstimator][ERROR]: Lidar+simple pose estimation not implemented";

    return resultPose;
}

communication::PoseCommunicator PoseEstimator::updateIMU()
{
    communication::PoseCommunicator resultPose {globComm->poseComm};

    return resultPose;
}


void PoseEstimator::calcSpeeds(communication::PoseCommunicator& pose)
{
    double timeDiff {std::chrono::duration_cast<std::chrono::milliseconds>(pose.curRobotTime-pose.lastRobotTime).count()/1000.0};

    // Calc speeds in localTile coordinate system
    double xSpeed {(pose.robotFrame.transform.pos_x - pose.lastRobotFrame.transform.pos_x)/timeDiff};
    double ySpeed {(pose.robotFrame.transform.pos_y - pose.lastRobotFrame.transform.pos_y)/timeDiff};

    pose.robotSpeed.transform.rot_z = (pose.robotFrame.transform.rot_z - pose.lastRobotFrame.transform.rot_z)/timeDiff;
    
    // Transform the speeds to robot local coordinate system.
    double angle {pose.lastRobotFrame.transform.rot_z};
    pose.robotSpeed.transform.pos_x = xSpeed*cos(angle) + ySpeed*sin(angle);
    pose.robotSpeed.transform.pos_y = ySpeed*cos(angle) - xSpeed*sin(angle);

    // Alternative: Set the parent of the speed to localTileFrame, then set speeds in localTile coordinate system. Then transform (or at least get the transform) down into the robot coordinate system. Should do this exact calculation, and looks nicer. Probem with the children?

}


double PoseEstimator::wrapValue(double value, double lower, double upper)
{
    double diffCorr {upper-lower};
    while (value>=upper)
    {
        value-=diffCorr;
    }
    while (value<lower)
    {
        value+=diffCorr;
    }

    return value;
}

// void PoseEstimator::wrapValueSameScale(double& val1, double& val2, double lower, double upper)
// {
//     double diff {upper-lower};
//     val1 = wrapValue(val1, lower, upper);
//     val2 = wrapValue(val2, lower, upper);

//     if (abs(val2-val1) > diff/2.0)
//     {
//         if (val1<val2)
//         {
//             val1+=diff;
//         }
//         else
//         {
//             val2+=diff;
//         }
//     }
// }

void PoseEstimator::wrapVectorSameScale(std::vector<ConditionalAverageTerm>& vec, double lower, double upper)
{
    double diff {upper-lower};
    if (vec.size()==0)
    {
        return;
    }
    std::sort(vec.begin(), vec.end());
    // The index in vec which denotes the largest small number (before the gap)
    int lowerLimitIndex {-1};

    // Find the gap
    for (int i=0;i<vec.size()-1;++i)
    {
        if (vec.at(i+1).value-vec.at(i).value > diff/3.0)
        {
            // We have found the last number before the gap
            lowerLimitIndex = i;
            break;
        }
    }

    // Get the lower values up to the scale of the larger values
    for (int i=0;i<=lowerLimitIndex;++i)
    {
        vec.at(i).value += diff;
    }
}


void PoseEstimator::wrapPoseComm(communication::PoseCommunicator& poseComm)
{
    Transform ghostTf {};
    // Z
    poseComm.robotFrame.transform.rot_z = wrapValue(poseComm.robotFrame.transform.rot_z, minZRot, maxZRot);
    // poseComm.lastRobotFrame.transform.rot_z = wrapValue(poseComm.lastRobotFrame.transform.rot_z, minZRot, maxZRot);

    poseComm.robotFrame.transform.pos_x = wrapValue(poseComm.robotFrame.transform.pos_x, minXPos, maxXPos);
    // poseComm.lastRobotFrame.transform.pos_x = wrapValue(poseComm.lastRobotFrame.transform.pos_x, minXPos, maxXPos);
    
    poseComm.robotFrame.transform.pos_y = wrapValue(poseComm.robotFrame.transform.pos_y, minYPos, maxYPos);
    // poseComm.lastRobotFrame.transform.pos_y = wrapValue(poseComm.lastRobotFrame.transform.pos_y, minYPos, maxYPos);

}


void PoseEstimator::updatePoseComm(communication::PoseCommunicator& pose)
{
    // If it has never been used before, do not judge (because it will jump).
    if (pose.freshness>0)
    {
        return;
    }

    Transform ghostTf {};

    double rotDiff {pose.robotFrame.transform.rot_z - pose.lastRobotFrame.transform.rot_z};
    // Turn right
    if ( rotDiff > tileRotDiffThreshold)
    {
        std::cout << "Turned right ";
        ghostTf.rot_z -= M_PI_2;
        ghostTf.pos_y += GRID_SIZE;

        pose.robotFrame.transform.rot_z -= M_PI_2;
    }
    // Turn left
    else if (rotDiff < -tileRotDiffThreshold)
    {
        std::cout << "Turned left ";
        ghostTf.rot_z += M_PI_2;
        ghostTf.pos_x += GRID_SIZE;
        pose.robotFrame.transform.rot_z += M_PI_2;

    }


    double xDiff {pose.robotFrame.transform.pos_x - pose.lastRobotFrame.transform.pos_x};
    // std::cout << "xDiff: " << xDiff << " ";

    // Move left
    if (xDiff > tileTransXDiffThreshold)
    {
        std::cout << "Moved left ";
        ghostTf.pos_x -= GRID_SIZE;
        pose.robotFrame.transform.pos_x -= GRID_SIZE;
    }
    // Move right
    else if (xDiff < -tileTransXDiffThreshold)
    {
        std::cout << "Moved right ";
        ghostTf.pos_x += GRID_SIZE;
        pose.robotFrame.transform.pos_x += GRID_SIZE;
    }


    double yDiff {pose.robotFrame.transform.pos_y - pose.lastRobotFrame.transform.pos_y};
    // std::cout << "yDiff: " << yDiff << " ";

    // Move back
    if (yDiff > tileTransYDiffThreshold)
    {
        std::cout << "Moved back ";
        ghostTf.pos_y -= GRID_SIZE;
        pose.robotFrame.transform.pos_y -= GRID_SIZE;
    }
    // Move forward
    else if (yDiff < -tileTransYDiffThreshold)
    {
        std::cout << "Moved forward ";
        ghostTf.pos_y += GRID_SIZE;
        pose.robotFrame.transform.pos_y += GRID_SIZE;
    }

    std::cout << std::endl;


    // std::cout << "Before ghostMove: " << "\n"
    // << "robot: " << pose.robotFrame << "  "
    // << "lastRobot: " << pose.lastRobotFrame << "  "
    // << "tile: " << pose.localTileFrame << "  "
    // << "\n";
    // Carry out the ghostmove
    pose.localTileFrame.ghostMove(ghostTf);
    // std::cout << "After ghostMove: " << "\n"
    // << "robot: " << pose.robotFrame << "  "
    // << "lastRobot: " << pose.lastRobotFrame << "  "
    // << "tile: " << pose.localTileFrame << "  "
    // << "\n";
    // wrapPoseComm(pose);
    // std::cout << "After wrapping: " << "\n"
    // << "robot: " << pose.robotFrame << "  "
    // << "lastRobot: " << pose.lastRobotFrame << "  "
    // << "tile: " << pose.localTileFrame << "  "
    // << "\n";

    
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

    Tof::TofData td {sensors->tofs.tofData};
    ConditionalAverageTerm flDiff {td.fl.avg-lastTofFL, 1};
    ConditionalAverageTerm frDiff {td.fr.avg-lastTofFR, 1};
    ConditionalAverageTerm bDiff {td.b.avg-lastTofB, 1};

    lastTofFL = td.fl.avg;
    lastTofFR = td.fr.avg;
    lastTofB = td.b.avg;

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
    ConditionalAverageTerm result {0, 1};

    // Check if angle is too great
    if (abs(angle)>MAX_Z_ROTATION_Y_TOF_ABS)
    {
        std::cout << "Angle too large for ToF Y abs trans" << std::endl;
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors->tofs.tofData};
    // Get Y distances
    ConditionalAverageTerm flY {td.fl.avg*cos(angle), 1};
    ConditionalAverageTerm frY {td.fr.avg*cos(angle), 1};
    ConditionalAverageTerm bY {td.b.avg*cos(angle), 1};

    // Change to robot centre
    flY.value = GRID_SIZE - (flY.value+yoffset*cos(angle)+xoffset*sin(angle) + WALL_THICKNESS/2.0);
    frY.value = GRID_SIZE - (frY.value+yoffset*cos(angle)+xoffset*sin(angle) + WALL_THICKNESS/2.0);
    bY.value = bY.value+yoffset*cos(angle) + WALL_THICKNESS/2.0;

    // Average calculation preparation
    Average avg {};
    avg.terms.push_back(flY);
    avg.terms.push_back(frY);
    avg.terms.push_back(bY);

    for (auto& term : avg.terms)
    {
        // Wrap values to local tile
        // std::cout << "Value is: " << term.value;

        // Check if sensor values can be used
        if (term.value > MAX_TOF_Y_DIST_ABS)
        {
            // std::cout << "Rejected due to too large distance" << std::endl;
            // Reject the term/value
            term.weight = 0;
        }
        else
        {
            // Accept the term/value
            term.weight = 1;
        }
        term.value = wrapValue(term.value, minYPos, maxYPos);
        // std::cout << "  After wrapping: " << term.value << std::endl;
    }

    // Fix wrapping mismatch
    avg.stripZeroWeight();
    wrapVectorSameScale(avg.terms, minYPos, maxYPos);

    try
    {
        result.value = avg.calc();
    }
    catch(std::runtime_error& e)
    {
        std::cerr << e.what() << '\n';
        // If not usable, do not use
        result.weight = 0;
    }
    
    // std::cout << "Result weight is: " << result.weight << std::endl;

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

    Tof::TofData td {sensors->tofs.tofData};
    double x1 {};
    double x2 {};

    if (getIsTofXLeft())
    {
        x1 = getCentredistanceFromTwoTof(td.lf.avg, td.lb.avg, TOF_SX_OFFSET, angle) + WALL_THICKNESS/2.0;
        x1 = wrapValue(x1, minXPos, maxXPos);
        if (getIsTofXRight())
        {
            x2 = GRID_SIZE - (getCentredistanceFromTwoTof(td.rf.avg, td.rb.avg, TOF_SX_OFFSET, angle) + WALL_THICKNESS/2.0);
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
        result.value = GRID_SIZE - (getCentredistanceFromTwoTof(td.rf.avg, td.rb.avg, TOF_SX_OFFSET, angle) + WALL_THICKNESS/2.0);
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

    #warning We want to check here so that we do not mess up when turning in enclosed space, but if we have lost tracking we will not be able to pick it up even if we are straight.
    if (abs(curAng)>MAX_ZROT_XTRANS_TOF_ABS)
    {
        std::cout << "Angle too large" << std::endl;
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors->tofs.tofData};
    double angle1 {};
    double angle2 {};
    if (getIsTofXLeft())
    {
        angle1 = getAngleFromTwoTof(td.lb.avg, td.lf.avg, TOF_SY_OFFSET*2);
        angle1 = wrapValue(angle1, minZRot, maxZRot);
        // std::cout << "Left: " << angle1;
        if (getIsTofXRight())
        {
            angle2 = -getAngleFromTwoTof(td.rb.avg, td.rf.avg, TOF_SY_OFFSET*2);
            angle2 = wrapValue(angle2, minZRot, maxZRot);
            // std::cout << ", Right: " << angle2;
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
        result.value = -getAngleFromTwoTof(td.rb.avg, td.rf.avg, TOF_SY_OFFSET*2);
        result.value = wrapValue(result.value, minZRot, maxZRot);
        // std::cout << ", Right: " << result.value;
        result.weight = 1;
    }
    else
    {
        // std::cerr << "Cannot compute ToF Z angle without walls";
        result.weight = 0;
    }

    // std::cout << "ToF Z Rot calculation done with result: " << result.value << ". Weight: " << result.weight << std::endl;

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
    double angle {sensors->imu0.angles.z};
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
    Tof::TofData td {sensors->tofs.tofData};

    if (td.lf.cur>WALL_PRESENCE_THRESHOLD_SENSOR || td.lf.avg>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    if (td.lb.cur>WALL_PRESENCE_THRESHOLD_SENSOR || td.lb.avg>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    return true;

}

bool PoseEstimator::getIsTofXRight()
{
    Tof::TofData td {sensors->tofs.tofData};

    if (td.rf.cur>WALL_PRESENCE_THRESHOLD_SENSOR || td.rf.avg>WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return false;
    }

    if (td.rb.cur>WALL_PRESENCE_THRESHOLD_SENSOR || td.rb.avg>WALL_PRESENCE_THRESHOLD_SENSOR)
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
