#include "localNav/PathFollower.h"

PathFollower::PathFollower(communication::Communicator* globComm, PiAbstractor* pAbs)
    : driver {globComm}
{
    this->globComm = globComm;
    // targetPoint.setParentTS(&(globComm->poseComm.localTileFrame));
    this->piAbs = pAbs;
    ledController.init(piAbs);

    readPidFromFile();

}

PathFollower::~PathFollower()
{
    driver.stop();
}

double PathFollower::getRotSpeedDriving(int direction)
{
    if (direction>=0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }
    // std::cout << "xPos: " << globComm->poseComm.robotFrame.transform.pos_x << "  ";
    double wantedAngle {yPid.getCorrection(globPose.robotFrame.transform.pos_x)};
    wantedAngle = capYPidOutput(wantedAngle);
    // std::cout << "wantedAngle: " << wantedAngle << "angle: " << globComm->poseComm.robotFrame.transform.rot_z << "  ";
    angPid.setSetpoint(-direction*wantedAngle);
    double speedCorr {angPid.getCorrection(globPose.robotFrame.transform.rot_z)};
    // std::cout << "speedCorr:" << speedCorr << "\n";
    return speedCorr;
    // return 0;
}

double PathFollower::getTransSpeedDriving(int direction)
{
    if (direction>=0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }
    double driveSpeed {};
    // Simple slow-down when close by
    if (direction*distLeftToTarget<DRIVING_CLOSE_PID_THRESHOLD)
    {
        // Can replace with separate PID later (whose output would be used: driveSpeed = baseSpeed+correction))
        driveSpeed = direction*DRIVE_SPEED_SLOW;
    }
    else
    {
        driveSpeed = direction*DRIVE_SPEED_STANDARD;
    }

    driveTransSpeedPid.setSetpoint(driveSpeed);
    // std::cout << "driveSpeed: " << driveSpeed << "  speed: " << globComm->poseComm.robotSpeedAvg.transform.pos_y << "  ";
    double corr {driveTransSpeedPid.getCorrection(globPose.robotSpeedAvg.transform.pos_y)};
    // std::cout << "transSpeedCorr: " << corr << "\n";
    return driveSpeed+corr;
    // return 200;
}

double PathFollower::getRotSpeedTurning(int direction)
{
    if (direction>=0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }
    double turnSpeed {};

    if (angLeftToTarget<TURNING_CLOSE_PID_THRESHOLD)
    {
        // Can replace with separate PID later (see getTransSpeedDriving for inspiration)
        turnSpeed = direction*TURN_SPEED_SLOW;
    }
    else
    {
        turnSpeed = direction*TURN_SPEED_STANDARD;
    }

    // #warning changed PID stuff - could break. Check here if it broke
    turnRotSpeedPid.setSetpoint(turnSpeed);
    // std::cout << "direction fixed turnSpeed: " << direction*turnSpeed << "  speed: " << globComm->poseComm.robotSpeedAvg.transform.rot_z << "  ";
    double corr {turnRotSpeedPid.getCorrection(globPose.robotSpeedAvg.transform.rot_z)};
    // std::cout << "rotSpeedCorr: " << corr << "\n";
    return (turnSpeed+corr);

}

double PathFollower::getTransSpeedTurning()
{
    // turnTransSpeedPid.setSetpoint(0);
    // return turnTransSpeedPid.getCorrection(globComm->poseComm.robotSpeedAvg.transform.pos_y);
    return 0;
}

void PathFollower::setLinePos(double newYLine)
{
    // If it is this extreme, we cannot follow it
    // #warning line setting is limited here
    // if (newYLine>250 || newYLine<50)
    // {
    //     return;
    // }
    yPid.setSetpoint(newYLine);
}

void PathFollower::runLoop()
{
    abortMove = false;
    while (lopActive)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        checkAndHandleLop();
    }
    if (lopDeactivated)
    {
        lopDeactivated = false;
        // Special reinitialization
    }
    yPid.restartPID();
    angPid.restartPID();
    driveTransSpeedPid.restartPID();
    turnRotSpeedPid.restartPID();

    globPose = globComm->poseComm.copyData(true);

    communication::DriveCommand dC {};
    if (driveBackwardsBlacktile)
    {
        dC = communication::DriveCommand::driveBackward;
        setBackWardTargetPointTf();
    }
    else
    {
        dC = globComm->navigationComm.popCommand();
        if (dC!=communication::DriveCommand::noAction)
        {
            setTargetPointTf(dC);
            globComm->poseComm.flushPose();
        }
    }

    switch(dC)
    {
        case communication::DriveCommand::driveForward:
            std::cout << "[PathFollower] driveForward\n";
            globComm->tileInfoComm.startDrive();
            setLinePos(GRID_SIZE/2.0);
            // alignAngle(true);
            // setTargetPointTf(dC);
            drive(1);
            // We are ready to update data
            if (!abortMove)
            {
                globComm->tileInfoComm.setReadyForFill();
                std::cout << "[PathFollower] Drove forward-----------------------------------------------------------------\n";
            }
            else
            {
                std::cout << "[PathFollower] Aborted drive step\n";
            }
            break;

        case communication::DriveCommand::driveBackward:
            std::cout << "[PathFollower] driveBackward\n";
            setLinePos(GRID_SIZE/2.0);
            // if (!driveBackwardsBlacktile)
            // {
            //     // Only align if not driving back due to black tile
            //     alignAngle(true);
            // }
            // Set target point again because it was reset by alignAngle
            if (!driveBackwardsBlacktile)
            {
                setTargetPointTf(dC);
            }
            drive(-1);

            if (!abortMove)
            {
                globComm->poseComm.flushPose();
                globComm->tileInfoComm.setReadyForFill();
                std::cout << "[PathFollower] Drove back------------------------------------------------------------------------\n";
            }
            // Reset driveBackwardsBlacktile
            driveBackwardsBlacktile = false;
            break;
        
        case communication::DriveCommand::turnLeft:
            std::cout << "[PathFollower] turnLeft\n";
            turn(1);
            alignAngle(false);
            std::cout << "[PathFollower] Turned left-----------------------------------------------------------------\n";
            break;

        case communication::DriveCommand::turnRight:
            std::cout << "[PathFollower] turnRight\n";
            turn(-1);
            alignAngle(false);
            std::cout << "[PathFollower] Turned right-----------------------------------------------------------------\n";
            break;

        case communication::DriveCommand::turnBack:
            turn(1);
            setTargetPointTf(communication::DriveCommand::turnLeft);
            turn(1);
            break;
        
        case communication::DriveCommand::init:
            std::cout << "[PathFollower] init\n";
            globComm->tileInfoComm.startDrive();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            globComm->tileInfoComm.setReadyForFill();
            break;

            
        default:
            checkAndHandlePanic();
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
            // std::cerr << "Cannot yet execute this DriveCommand" << std::endl;
            break;
    }

    driver.stop();
}

void PathFollower::drive(int direction)
{
    bool finished {false};
    // int direction = getDriveDirection();
    globComm->poseComm.setTurning(false);
    globComm->poseComm.setDriving(true);
    while(!finished)
    {
        if (globComm->poseComm.getUpdated())
        {
            // Create local copy of pose
            globPose = globComm->poseComm.copyData(true);
            checkAndHandlePanic();
            distLeftToTarget = getDistLeftToTarget();
            std::cout << "distLeftToTarget: " << distLeftToTarget << "\n";
            driver.calcSpeeds(getTransSpeedDriving(direction), getRotSpeedDriving(direction));
            driver.setSpeeds();
            finished = checkIsFinishedDriving(direction);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    driver.stop();
    globComm->poseComm.setDriving(false);
}

void PathFollower::alignAngle(bool usePidAngle)
{
    yPid.restartPID();
    angPid.restartPID();
    driveTransSpeedPid.restartPID();
    turnRotSpeedPid.restartPID();
    turnTransSpeedPid.restartPID();

    if (usePidAngle)
    {
        setTargetPointTf(Transform{static_cast<double>(GRID_SIZE/2), static_cast<double>(GRID_SIZE/2), 0, 0, 0, yPid.getCorrection(globPose.robotFrame.transform.rot_z)});
    }
    else
    {
        setTargetPointTf(Transform{static_cast<double>(GRID_SIZE/2), static_cast<double>(GRID_SIZE/2), 0, 0, 0, 0});
    }

    // turn();

}

void PathFollower::turn(int direction)
{
    bool finished {false};
    // int direction = getTurnDirection();
    globComm->poseComm.setTurning(true);
    globComm->poseComm.setDriving(false);
    while (!finished)
    {
        if (globComm->poseComm.getUpdated())
        {
            globPose = globComm->poseComm.copyData(true);
            checkAndHandlePanic();
            angLeftToTarget = getAngLeftToTarget();
            std::cout << "angLeftToTarget: " << angLeftToTarget << "\n";
            driver.calcSpeeds(getTransSpeedTurning(), getRotSpeedTurning(direction));
            driver.setSpeeds();
            finished = checkIsFinishedTurning(direction);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    driver.stop();
    globComm->poseComm.setTurning(false);
}

void PathFollower::runLoopLooper()
{
    driver.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while(true)
    {
        runLoop();
    }
}

bool PathFollower::checkIsFinishedDriving(int direction)
{
    if (direction>=0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }

    if (abortMove)
    {
        return true;
    }

    // distLeftToTarget is updated in another loop, as it is used both for this and for speed control

    // If driving forward and there is an obstacle in front
    if (direction==1 && globComm->poseComm.getFrontObstacleDist() !=-1)
    {
        if (globComm->poseComm.getFrontObstacleDist()<FRONT_OBSTACLE_DRIVE_STOP_THRESHOLD && globPose.robotSpeedAvg.transform.pos_y >= (0.69*DRIVE_SPEED_SLOW))
        {
            std::cout << "[PathFollower] Obstacle in front\n";
            return true;
        }
        else if (globComm->poseComm.getFrontObstacleDist()<FRONT_OBSTACLE_STANDING_STOP_THRESHOLD)
        {
            std::cout << "[PathFollower] Obstacle in front\n";
            return true;
        }
        else
        {
            return false;
        }
    }

    if (direction*distLeftToTarget < DRIVE_STOP_THRESHOLD)
    {
        return true;
    }
    else
    {
        return false;
    }

}

bool PathFollower::checkIsFinishedTurning(int direction)
{
    if (direction>=0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }

    if (abortMove)
    {
        return true;
    }

    // angLeftToTarget is updated in another loop, as it is used both for this and for speed control

    if (direction*angLeftToTarget < TURN_STOP_THRESHOLD)
    {
        return true;
    }
    else
    {
        return false;
    }
}



void PathFollower::setTargetPointTf(communication::DriveCommand dC)
{
    Transform resultTf {static_cast<double>(GRID_SIZE)/2, static_cast<double>(GRID_SIZE)/2, 0, 0, 0, 0};
    switch (dC)
    {
        case communication::DriveCommand::driveForward:
            resultTf.pos_y += GRID_SIZE;
            break;

        case communication::DriveCommand::driveBackward:
            resultTf.pos_y -= GRID_SIZE;
            break;

        case communication::DriveCommand::turnLeft:
            resultTf.rot_z += M_PI_2;
            break;
        
        case communication::DriveCommand::turnRight:
            resultTf.rot_z -= M_PI_2;
            break;
        
        case communication::DriveCommand::turnBack:
            resultTf.rot_z += M_PI;
            break;

        default:
            // std::cerr << "Cannot set target point with this DriveCommand";
            break;
    }

    setTargetPointTf(resultTf);
    
}

void PathFollower::setTargetPointTf(Transform tf)
{
    // std::cout << "PathFollower setTargetPointTf borrowing...";
    communication::PoseDataSyncBlob poseBlob {globComm->poseComm.borrowData()};
    // std::cout << "done\n";
    poseBlob.targetFrame.transform = tf;
    // std::cout << "PathFollower setTargetPointTf giving back...";
    globComm->poseComm.giveBackData(poseBlob, false);
    // std::cout << "done\n";
    // globComm->poseComm.setTargetFrameTransformTS(tf);
}

void PathFollower::setBackWardTargetPointTf()
{
    Transform resultTf {static_cast<double>(GRID_SIZE)/2, static_cast<double>(GRID_SIZE)/2, 0, 0, 0, 0};

    if (globComm->poseComm.hasDrivenStep())
    {
        resultTf.pos_y -= GRID_SIZE;
    }
    // Else, do nothing as resultTf is already on the correct tile.
    
    // Update the targetPoint
    setTargetPointTf(resultTf);
}

int PathFollower::getTurnDirection()
{
    if (globPose.getTargetFrame().transform.rot_z - globPose.robotFrame.transform.rot_z > 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

int PathFollower::getDriveDirection()
{
    if (globPose.getTargetFrame().transform.pos_y - globPose.robotFrame.transform.pos_y > 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

double PathFollower::getDistLeftToTarget()
{
    // Transform targetPointTf {targetPoint.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 1)};
    // return targetPointTf.pos_y;

    
    return globPose.getTargetFrame().transform.pos_y - globPose.robotFrame.transform.pos_y;
}

double PathFollower::getAngLeftToTarget()
{
    // Transform targetPointTf {targetPoint.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 1)};
    double rotDiff {globPose.getTargetFrame().transform.rot_z - globPose.robotFrame.transform.rot_z};
    if (rotDiff > M_PI)
    {
        rotDiff -= 2*M_PI;
    }
    return rotDiff;
}

void PathFollower::readPidFromFile()
{
    std::ifstream file ("pid.txt", std::ios::in);
    if (!file.is_open())
    {
        std::cerr << "Could not open PID file\n";
        exit(1);
    }

    double kP {};
    double kI {};
    double kD {};

    file >> kP;
    file >> kI;
    file >> kD;
    std::cout << "Read for driveTransSpeedPid: kP=" << kP << " , kI=" << kI << " , kD=" << kD << "\n";
    driveTransSpeedPid.setCoeff(kP, kI, kD);

    file >> kP;
    file >> kI;
    file >> kD;
    std::cout << "Read for yPid: kP=" << kP << " , kI=" << kI << " , kD=" << kD << "\n";
    yPid.setCoeff(kP, kI, kD);

    file >> kP;
    file >> kI;
    file >> kD;
    std::cout << "Read for angPid: kP=" << kP << " , kI=" << kI << " , kD=" << kD << "\n";
    angPid.setCoeff(kP, kI, kD);

    file >> kP;
    file >> kI;
    file >> kD;
    std::cout << "Read for turnTransSpeedPid: kP=" << kP << " , kI=" << kI << " , kD=" << kD << "\n";
    turnTransSpeedPid.setCoeff(kP, kI, kD);

    file >> kP;
    file >> kI;
    file >> kD;
    std::cout << "Read for turnRotSpeedPid: kP=" << kP << " , kI=" << kI << " , kD=" << kD << "\n";
    turnRotSpeedPid.setCoeff(kP, kI, kD);
}




void PathFollower::checkAndHandlePanic()
{
    checkAndHandleLop();

    // Ramp
    if (globComm->panicFlagComm.readFlagFromThread(communication::PanicFlags::onRamp, communication::ReadThread::localNav))
    {
        #warning not done
    }

    // Black tile
    if (globComm->panicFlagComm.readFlagFromThread(communication::PanicFlags::sawBlackTile, communication::ReadThread::localNav))
    {
        handleBlackTile();
    }

    // Victim
    if (globComm->panicFlagComm.readFlagFromThread(communication::PanicFlags::victimDetected, communication::ReadThread::localNav))
    {
        handleVictim();
    }
}


void PathFollower::handleVictim()
{
    driver.stop();

    // Do the blinking
    ledController.blinkLedVictimFound();
    ledController.waitForFinish();

    // Reset so that we can continue
    yPid.restartPID();
    angPid.restartPID();
    driveTransSpeedPid.restartPID();
    turnRotSpeedPid.restartPID();
}

void PathFollower::handleBlackTile()
{
    // Do not do anything if we are already reversing due to black tile or if we are not driving (i.e. turning)
    if (driveBackwardsBlacktile || !globComm->poseComm.getDriving())
    {
        return;
    }
    driver.stop();

    std::cout << "[PathFollower] Saw black tile - going back to safety\n";

    // Reverse
    driveBackwardsBlacktile = true;
    abortMove = true;
}

void PathFollower::checkAndHandleLop()
{
    // LOP
    if (globComm->panicFlagComm.readFlagFromThread(communication::PanicFlags::lackOfProgressActivated, communication::ReadThread::localNav))
    {
        handleLOP();
    }

    if (globComm->panicFlagComm.readFlagFromThread(communication::PanicFlags::lackOfProgressDeactivated, communication::ReadThread::localNav))
    {
        handleLopDeactivated();
    }
}

void PathFollower::handleLOP()
{
    driver.stop();

    abortMove = true;
    lopActive = true;
}

void PathFollower::handleLopDeactivated()
{
    lopActive = false;
    lopDeactivated = true;
}

double PathFollower::capYPidOutput(double output)
{
    if (output>YPID_MAX_MIN_OUTPUT_ANGLE)
    {
        output = YPID_MAX_MIN_OUTPUT_ANGLE;
    }
    else if (output < -YPID_MAX_MIN_OUTPUT_ANGLE)
    {
        output = -YPID_MAX_MIN_OUTPUT_ANGLE;
    }
    else
    {
        // Do nothing
    }
    return output;
}