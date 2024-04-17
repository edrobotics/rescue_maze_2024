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


void PoseEstimator::flush(FusionGroup fgroup)
{
    for (int i=0;i<8;++i)
    {
        update(fgroup, false);
    }
    update(fgroup, true);
}

void PoseEstimator::runLoopLooper(communication::Communicator* globComm)
{
    // std::cout << "Started runLoopLooper" << "\n";
    this->globComm = globComm;
    flush(FusionGroup::fg_simple);
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
    // if (globComm->poseComm.getShouldFlushPose())
    // {
    //     flush(fusionGroup);
    // }
    update(fusionGroup, true);
}

void PoseEstimator::update(FusionGroup fgroup, bool doUpdate)
{
    // std::cout << "In update" << std::endl;
    sensors->update(true);
    // std::cout << "updated sensors  ";
    // Should copy the tree of poseComm into poseResult
    // std::cout << "Creating poseResult... ";
    communication::PoseCommunicator poseResult {};
    // std::cout << "done\n";
    // std::cout << "lock poseComm mutex... ";
    // Lock poseComm to prevent concurrency issues with the targetPoint
    globComm->poseComm.mtx_general.lock();
    isTurning = globComm->poseComm.getTurning();
    isDriving = globComm->poseComm.getDriving();
    // std::cout << "done\n";
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
    // std::cout << "before updatePoseComm\n";
    updatePoseComm(poseResult);

    // std::cout << "In poseEstimator, before calcSpeeds\n";
    calcSpeeds(poseResult);
    poseResult.updated = true;

    if (doUpdate)
    {
        // Write the new pose with only changes.
        #warning concurrency issues?
        globComm->poseComm = poseResult;
    }
    // Unlock poseComm to allow access to targetPoint again.
    globComm->poseComm.mtx_general.unlock();

    // std::cout << "robotSpeed: " << globComm->poseComm.robotSpeedAvg << "\n";
    std::cout << "robotFrame: " << globComm->poseComm.robotFrame << /*"  lastRobotFrame: " << globComm->poseComm.lastRobotFrame << */ "\n";
    // sensors->tofs.printVals(true);
    // sensors->imu0.printVals(true);

    updateTileProperties();
}





#warning currently a lot of code without the right conditional logic. Do not expect to work at all
communication::PoseCommunicator PoseEstimator::updateSimple()
{
    // communication::PoseCommunicator lastPose {globComm->poseComm};
    // std::cout << "before resultPose copy from poseComm... ";
    communication::PoseCommunicator resultPose {globComm->poseComm};
    // std::cout << "done\n";
    resultPose.lastRobotFrame = resultPose.robotFrame;
    // std::cout << "lastRobotFrame: " << resultPose.lastRobotFrame << "\n";
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
    ConditionalAverageTerm imuRot {getIMURotDiff()};

    if (wheelRot.weight != 0 && imuRot.weight != 0)
    {
        // std::cout << "wheel=" << std::setprecision(3) << abs(wheelRot.value);
        // std::cout << "  imu=" << std::setprecision(3) << abs(imuRot.value);
        // std::cout << "  diff=" << std::setprecision(3) << abs(imuRot.value-wheelRot.value) << "\n";
        if (abs(imuRot.value) < MAX_IMU_FROZEN_ANGLE && abs(imuRot.value-wheelRot.value) > MIN_IMU_WHEEL_DIFF_FROZEN)
        {
            // IMU freeze detected
            std::cout << "IMU FROZEN\n";

            // Invalidate the IMU readings
            imuRot.weight = 0;
        }
    }
    else
    {
        if (wheelRot.weight==0)
        {
            // std::cout << "wheel unusable\n";
        }
        if (imuRot.weight==0)
        {
            // std::cout << "imu unusable\n";
        }
    }

    wheelRot.value += resultPose.lastRobotFrame.transform.rot_z;
    rotAbs.terms.push_back(wheelRot);

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
        // std::cerr << e.what() << " : " << "Cannot compute robot angle" << '\n';
        #warning what to do here, with no angle? Also what to do for further angle requirements
    }

    // std::cout << "rotation done\n";
    
    
    // Translation, abs:
    Average transXAbs {};
    Average transYAbs {};

    transXAbs.terms.push_back(getTofXTrans(resultPose.lastRobotFrame.transform.rot_z));
    transYAbs.terms.push_back(getTofYTrans(resultPose.lastRobotFrame.transform.rot_z, TOF_FY_OFFSET, TOF_FX_OFFSET));

    ConditionalAverageTerm wheelTransX {getWheelTransDiff()};
    ConditionalAverageTerm wheelTransY {wheelTransX};
    wheelTransX.value = wheelTransX.value*sin(resultPose.lastRobotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_x;
    wheelTransY.value = wheelTransY.value*cos(resultPose.lastRobotFrame.transform.rot_z) + resultPose.lastRobotFrame.transform.pos_y;
    transXAbs.terms.push_back(wheelTransX);
    transYAbs.terms.push_back(wheelTransY);

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

    if (!isTurning)
    {
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
        // std::cout << "translation done\n";
    }


    return resultPose;

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

    CoordinateFrame newSpeed {nullptr};
    // Calc speeds in localTile coordinate system
    double xSpeed {(pose.robotFrame.transform.pos_x - pose.lastRobotFrame.transform.pos_x)/timeDiff};
    double ySpeed {(pose.robotFrame.transform.pos_y - pose.lastRobotFrame.transform.pos_y)/timeDiff};

    newSpeed.transform.rot_z = (pose.robotFrame.transform.rot_z - pose.lastRobotFrame.transform.rot_z)/timeDiff;
    
    // Transform the speeds to robot local coordinate system.
    double angle {pose.lastRobotFrame.transform.rot_z};
    newSpeed.transform.pos_x = xSpeed*cos(angle) + ySpeed*sin(angle);
    newSpeed.transform.pos_y = ySpeed*cos(angle) - xSpeed*sin(angle);

    pose.calcRobotSpeedAvg(newSpeed);
    
    // Debugging
    // std::cout << "timeDiff: " << timeDiff << "  ";
    // std::cout << "xSpeed: " << xSpeed << "  ";
    // std::cout << "realXSpeed: " << pose.robotSpeed.transform.pos_x << "  ";
    // std::cout << "ySpeed: " << ySpeed << "  ";
    // std::cout << "realYSpeed: " << pose.robotSpeed.transform.pos_y << "  ";
    // std::cout << "\n";


    // Alternative: Set the parent of the speed to localTileFrame, then set speeds in localTile coordinate system. Then transform (or at least get the transform) down into the robot coordinate system. Should do this exact calculation, and looks nicer. Probem with the children?
    // pose.robotSpeed.setParentTS(&(pose.localTileFrame));
    // pose.robotSpeed.transform.pos_x = xSpeed;
    // pose.robotSpeed.transform.pos_y = ySpeed;
    // pose.robotSpeed.transformLevelTo(&(pose.robotFrame), 1, 1);
    // pose.robotSpeed.transform.rot_z = (pose.robotFrame.transform.rot_z - pose.lastRobotFrame.transform.rot_z)/timeDiff;
    // #warning requires chnages in assignment constructor of posecomm to make the parent correct?

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
    for (int i=0;i<static_cast<int>(vec.size()-1);++i)
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
        if (!isDriving)
        {
            std::cout << "Turned right--------------------------------------\n";
            ghostTf.rot_z -= M_PI_2;
            ghostTf.pos_y += GRID_SIZE;
        }

        pose.robotFrame.transform.rot_z -= M_PI_2;
    }
    // Turn left
    else if (rotDiff < -tileRotDiffThreshold)
    {
        if (!isDriving)
        {
            std::cout << "Turned left--------------------------------------\n";
            ghostTf.rot_z += M_PI_2;
            ghostTf.pos_x += GRID_SIZE;
        }

        pose.robotFrame.transform.rot_z += M_PI_2;
    }



    double xDiff {pose.robotFrame.transform.pos_x - pose.lastRobotFrame.transform.pos_x};
    // std::cout << "xDiff: " << xDiff << " ";

    // Move left
    if (xDiff > tileTransXDiffThreshold)
    {
        std::cout << "Moved left---------------------------------------------\n";
        ghostTf.pos_x -= GRID_SIZE;
        pose.robotFrame.transform.pos_x -= GRID_SIZE;
    }
    // Move right
    else if (xDiff < -tileTransXDiffThreshold)
    {
        std::cout << "Moved right---------------------------------------------\n";
        ghostTf.pos_x += GRID_SIZE;
        pose.robotFrame.transform.pos_x += GRID_SIZE;
    }


    double yDiff {pose.robotFrame.transform.pos_y - pose.lastRobotFrame.transform.pos_y};
    // std::cout << "yDiff: " << yDiff << " ";

    // Move back
    if (yDiff > tileTransYDiffThreshold)
    {
        std::cout << "Moved back------------------------------------------------\n";
        ghostTf.pos_y -= GRID_SIZE;
        pose.robotFrame.transform.pos_y -= GRID_SIZE;
    }
    // Move forward
    else if (yDiff < -tileTransYDiffThreshold)
    {
        std::cout << "Moved forward-----------------------------------------------\n";
        ghostTf.pos_y += GRID_SIZE;
        pose.robotFrame.transform.pos_y += GRID_SIZE;
    }

    // std::cout << std::endl;


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
    // MotorControllers::Distances motorDistances {sensors->motors.getDistances()};
    // motorDistanceDiffs.lf = motorDistances.lf - lastMotorDistances.lf;
    // motorDistanceDiffs.lb = motorDistances.lb - lastMotorDistances.lb;
    // motorDistanceDiffs.rf = motorDistances.rf - lastMotorDistances.rf;
    // motorDistanceDiffs.rb = motorDistances.rb - lastMotorDistances.rb;


    // lastMotorDistances = motorDistances;

    motorDistanceDiffs = sensors->motors.getDistances();
    // std::cout << "Motor distance diffs: lf=" << motorDistanceDiffs.lf << "  ";
    // std::cout << "lb=" << motorDistanceDiffs.lb << "  ";
    // std::cout << "rf=" << motorDistanceDiffs.rf << "  ";
    // std::cout << "rb=" << motorDistanceDiffs.rb << "\n";
}

ConditionalAverageTerm PoseEstimator::getWheelTransDiff()
{
    ConditionalAverageTerm result {0, 0};

    // if (isTurning)
    // {
    //     return result;
    // }

    Average avg {};

    avg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.lf), 1});
    avg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.lb), 1});
    avg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.rf), 1});
    avg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.rb), 1});

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
        std::cout << "Could not use wheel odom diff\n";
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
        // std::cout << "Angle too large for ToF Y abs trans" << std::endl;
        result.weight = 0;
        return result;
    }

    Tof::TofData td {sensors->tofs.tofData};
    // Get Y distances
    ConditionalAverageTerm flY {td.fl.avg*cos(angle), 1};
    ConditionalAverageTerm frY {td.fr.avg*cos(angle), 1};
    ConditionalAverageTerm bY {td.b.avg*cos(angle), 1};

    // Change to robot centre
    flY.value = (flY.value+yoffset*cos(angle)+xoffset*sin(angle) + WALL_THICKNESS/2.0);
    frY.value = (frY.value+yoffset*cos(angle)+xoffset*sin(angle) + WALL_THICKNESS/2.0);
    bY.value = bY.value+yoffset*cos(angle) + WALL_THICKNESS/2.0;

    // Average calculation preparation
    Average avg {};
    avg.terms.push_back(flY);
    avg.terms.push_back(frY);
    avg.terms.push_back(bY);
    // std::cout << "bY: " << bY.value << "  ";
    // std::cout << "b.avg: " << td.b.avg << "  ";
    // std::cout << "b.cur: " << td.b.cur << "  ";
    // std::cout << "\n";

    // Disqualify too large values
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
            // std::cout << "term.value: " << term.value;
            // std::cout << "flY: " << flY.value << "  ";
            // std::cout << "frY: " << frY.value << "  ";
            term.weight = 1;
        }
    }

    // Invert the front sensors
    avg.terms.at(0).value = GRID_SIZE - avg.terms.at(0).value;
    avg.terms.at(1).value = GRID_SIZE - avg.terms.at(1).value;

    for (auto& term : avg.terms)
    {
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
        // std::cerr << e.what() << '\n';
        // If not usable, do not use
        result.weight = 0;
    }

    globComm->poseComm.setFrontObstacleDist(getFrontObstacleDist(td));

    
    // std::cout << "Result weight is: " << result.weight << std::endl;

    return result;
}


ConditionalAverageTerm PoseEstimator::getTofXTrans(double angle)
{
    ConditionalAverageTerm result {0, 0};

    bool turning {isTurning};

    if ((abs(angle)>MAX_ZROT_XTRANS_TOF_ABS && !turning) || (abs(angle)>MAX_ZROT_XTRANS_TOF_ABS_TURNING && turning))
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

    bool turning {isTurning};

    #warning We want to check here so that we do not mess up when turning in enclosed space, but if we have lost tracking we will not be able to pick it up even if we are straight.
    if ((abs(curAng)>MAX_ZROT_XTRANS_TOF_ABS && !turning) || (abs(curAng)>MAX_ZROT_XTRANS_TOF_ABS_TURNING && turning))
    {
        std::cout << "Angle too large for tof z rot" << std::endl;
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

    Average leftAvg {};
    Average rightAvg {};

    leftAvg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.lf), 1});
    leftAvg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.lb), 1});

    rightAvg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.rf), 1});
    rightAvg.terms.push_back(ConditionalAverageTerm{static_cast<double>(motorDistanceDiffs.rb), 1});


    // Filter out bad values
    for (auto& term : leftAvg.terms)
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
    for (auto& term : rightAvg.terms)
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
    
    double leftDist {};
    double rightDist {};
    int succeeded {0};

    try
    {
        leftDist = leftAvg.calc();
        ++succeeded;
    }
    catch (std::runtime_error& e)
    {
        // No usable data, so do not use
    }

    try
    {
        rightDist = rightAvg.calc();
        ++succeeded;
    }
    catch (std::runtime_error& e)
    {
        // No usable data, so do not use
    }

    if (succeeded==2)
    {
        result.weight = 1;

        // Path along circumference
        result.value = (rightDist-leftDist)/2.0;

        // Path to angle - what we want in the end
        result.value = result.value/WHEEL_TURN_RADIUS;
    }
    else
    {
        std::cout << "Could not use wheel diff\n";
        result.weight = 0;
    }

    return result;
}


ConditionalAverageTerm PoseEstimator::getIMURotDiff()
{
    ConditionalAverageTerm result {0, 0};
    double angle {sensors->imu0.angles.z};

    bool imuReset {sensors->imu0.getWasReset()};
    
    // Reset handling
    if (lastImuWasReset && !imuReset)
    {
        // Last sensor value was reset but not this one => startup imu calc again
        // std::cout << "Staring IMU after reset\n";
        // Communicate no-use
        result.value = 0;
        result.weight = 0;
        // Fix so it works next iteration. This is the first good iteration
        lastImuAngle = angle;
        lastImuWasReset = false;
        
        return result;
    }
    else if (imuReset)
    {
        // Reset detected this run
        // std::cout << "IMU RESET!" << "\n";
        lastImuWasReset = true;
        // Communicate no-use
        result.value = 0;
        result.weight = 0;
        return result;
    }


    result.value = angle-lastImuAngle;

    // Prepare for next iteration
    lastImuAngle = angle;
    // std::cout << "IMU angle: " << angle << "  ";

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


double PoseEstimator::getFrontObstacleDist(Tof::TofData td)
{
    int blockedSum {0};
    int blockedValueSum {0};

    if (td.fl.avg<FRONT_OBSTACLE_DETECTION_THRESHOLD)
    {
        blockedValueSum += td.fl.avg;
        ++blockedSum;
    }
    if (td.fl.cur<FRONT_OBSTACLE_DETECTION_THRESHOLD)
    {
        blockedValueSum += td.fl.cur;
        ++blockedSum;
    }

    if (td.fr.avg<FRONT_OBSTACLE_DETECTION_THRESHOLD)
    {
        blockedValueSum += td.fr.avg;
        ++blockedSum;
    }
    if (td.fr.cur<FRONT_OBSTACLE_DETECTION_THRESHOLD)
    {
        blockedValueSum += td.fr.cur;
        ++blockedSum;
    }

    if (blockedSum>=3)
    {
        // Return the average blocked distance
        return static_cast<double>(blockedValueSum)/static_cast<double>(blockedSum);
    }
    else
    {
        // Return -1 - means that no obstacle is there
        return -1;
    }
}


void PoseEstimator::updateTileProperties()
{
    if (globComm->tileInfoComm.getDriveStarted())
    {
        globComm->poseComm.startLocalTileFrame = globComm->poseComm.localTileFrame.getWithoutChildren();
    }

    // Check if requested
    if (!globComm->tileInfoComm.getIsReadyForFill())
    {
        return;
    }

    communication::TileDriveProperties tProp {};

    tProp.wallsOnNewTile = getWallStates();
    tProp.tileColourOnNewTile = getTileColour();
    tProp.droveTile = getHasLocalTileMoved();
    tProp.usedRamp = getDroveRamp();

    globComm->tileInfoComm.setNewTileProperties(tProp);
}


std::vector<communication::Walls> PoseEstimator::getWallStates()
{
    Tof::TofData td {sensors->tofs.tofData};
    std::vector<communication::Walls> result {};
    std::cout << "[PoseEstimator] Wallstates: ";
    std::cout << "left:";
    if (getLeftWallPresent(td))
    {
        result.push_back(communication::Walls::LeftWall);
        std::cout << "1";
    }
    else
    {
        std::cout << "0";
    }
    std::cout << "  right:";

    if (getRightWallPresent(td))
    {
        result.push_back(communication::Walls::RightWall);
        std::cout << "1";
    }
    else
    {
        std::cout << "0";
    }

    std::cout << "  front:";
    if (getFrontWallPresent(td))
    {
        result.push_back(communication::Walls::FrontWall);
        std::cout << "1";
    }
    else
    {
        std::cout << "0";
    }

    std::cout << "  back:";
    if (getBackWallPresent(td))
    {
        result.push_back(communication::Walls::BackWall);
        std::cout << "1";
    }
    else
    {
        std::cout << "0";
    }

    std::cout << std::endl;


    return result;
}

TileColours PoseEstimator::getTileColour()
{
    #warning not yet implemented
    return TileColours::White;
}

bool PoseEstimator::getHasLocalTileMoved()
{
    if (globComm->poseComm.hasDrivenStep())
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool PoseEstimator::getDroveRamp()
{
    #warning unimplemented
    return false;
}

bool PoseEstimator::getLeftWallPresent(Tof::TofData td)
{
    if (td.lf.avg<WALL_PRESENCE_THRESHOLD_SENSOR && td.lb.avg<WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return true;
    }
    else
    {
        return false;
    }

}

bool PoseEstimator::getRightWallPresent(Tof::TofData td)
{
    if (td.rf.avg<WALL_PRESENCE_THRESHOLD_SENSOR && td.rb.avg<WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PoseEstimator::getFrontWallPresent(Tof::TofData td)
{
    if (td.fl.avg<WALL_PRESENCE_THRESHOLD_SENSOR && td.fr.avg<WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return true;
    }
    else
    {
        return false;
    }

}

bool PoseEstimator::getBackWallPresent(Tof::TofData td)
{
    if (td.b.avg<WALL_PRESENCE_THRESHOLD_SENSOR)
    {
        return true;
    }
    else
    {
        return false;
    }
}
