#include "localNav/PathFollower.h"

PathFollower::PathFollower(communication::Communicator* globComm)
    : driver {globComm}
{
    this->globComm = globComm;
    // targetPoint.setParentTS(&(globComm->poseComm.localTileFrame));
}

PathFollower::~PathFollower()
{
    driver.stop();
}

double PathFollower::getRotSpeedDriving()
{
    double wantedAngle {yPid.getCorrection(globComm->poseComm.robotFrame.transform.pos_x)};
    angPid.setSetpoint(wantedAngle);
    double speedCorr {angPid.getCorrection(globComm->poseComm.robotFrame.transform.rot_z)};
    return speedCorr;
}

double PathFollower::getTransSpeedDriving()
{
    #warning not yet implemented correctly
    return 200;
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
    return direction*M_PI;
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
    yPid.restartPID();
    angPid.restartPID();
    communication::DriveCommand dC {globComm->navigationComm.popCommand()};
    // Set the target to go to
    setTargetPointTf(dC);
    switch(dC)
    {
        case communication::DriveCommand::driveForward:
            setLinePos(GRID_SIZE/2.0);
            drive(1);
            break;
        
        case communication::DriveCommand::turnLeft:
            turn(1);
            break;

        case communication::DriveCommand::turnRight:
            turn(-1);
            break;

        default:
            std::cerr << "Cannot yet execute this DriveCommand" << std::endl;
            break;
    }

    driver.stop();
}

void PathFollower::drive(int direction)
{
    bool finished {false};
    while(!finished)
    {
        distLeftToTarget = getDistLeftToTarget();
        std::cout << "distLeftToTarget: " << distLeftToTarget << "\n";
        driver.calcSpeeds(getTransSpeedDriving(), getRotSpeedDriving());
        driver.setSpeeds();
        finished = checkIsFinishedDriving(direction);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PathFollower::turn(int direction)
{
    bool finished {false};
    while (!finished)
    {
        angLeftToTarget = getAngLeftToTarget();
        driver.calcSpeeds(0, getRotSpeedTurning(direction));
        driver.setSpeeds();
        finished = checkIsFinishedTurning(direction);
    }
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

    // distLeftToTarget is updated in another loop, as it is used both for this and for speed control

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
    Transform resultTf {GRID_SIZE/2, GRID_SIZE/2, 0, 0, 0, 0};
    switch (dC)
    {
        case communication::DriveCommand::driveForward:
            resultTf.pos_y += GRID_SIZE;
            break;
        
        case communication::DriveCommand::turnLeft:
            resultTf.rot_z += M_PI_2;
            break;
        
        case communication::DriveCommand::turnRight:
            resultTf.rot_z -= M_PI_2;
            break;

        default:
            std::cerr << "Cannot set target point with this DriveCommand";
            break;
    }
    
    globComm->poseComm.setTargetFrameTransformTS(resultTf);
    
}


double PathFollower::getDistLeftToTarget()
{
    // Transform targetPointTf {targetPoint.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 1)};
    // return targetPointTf.pos_y;
    
    return globComm->poseComm.getTargetFrameTS().transform.pos_y - globComm->poseComm.robotFrame.transform.pos_y;
}

double PathFollower::getAngLeftToTarget()
{
    // Transform targetPointTf {targetPoint.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 1)};
    double rotDiff {globComm->poseComm.getTargetFrameTS().transform.rot_z - globComm->poseComm.robotFrame.transform.rot_z};
    if (rotDiff> M_PI)
    {
        rotDiff -= 2*M_PI;
    }
    return rotDiff;
}




// void PathFollower::setPath(Path path)
// {
//     // Set the path
//     this->path = path;

//     // Update the last known frame 
//     lastKnownFrame.setParentTS(&(path.parentFrame));
//     // Reset the last known frame
//     lastKnownFrame.transform = Transform{};
// }



// bool PathFollower::setLookaheadDistance(double distance)
// {
//     // Too low
//     if (distance < minLookaheadDistance)
//     {
//         lookaheadDistance = minLookaheadDistance;
//         return false;
//     }

//     // Too high
//     if (distance > maxLookaheadDistance)
//     {
//         lookaheadDistance = maxLookaheadDistance;
//         return false;
//     }

//     // Valid
//     lookaheadDistance = distance;
//     return true;
// }


// double PathFollower::getLookaheadAngle()
// {
//     Transform tf {getLookaheadTF()};
//     return atan2(tf.pos_y, tf.pos_x)-M_PI_2;
// }

// Transform PathFollower::getLookaheadTF()
// {
//     return lookaheadCf.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 2);
// }


// #warning handle division by zero somehow. Happens if two transforms are equal. Check for that instead or do that higher up in the chain?
// Transform PathFollower::getLookaheadTF()
// {
//     int index {pathSegmentIndex};
//     #warning not protected if index is out of bounds
//     CoordinateFrame cf1 {path.keyFrames.at(index).frame};
//     CoordinateFrame cf2 {path.keyFrames.at(index+1).frame};

//     Transform tf1 {cf1.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 2)};
//     Transform tf2 {cf2.getTransformLevelTo(&(globComm->poseComm.robotFrame), 1, 2)};

//     double dx {tf2.pos_x-tf1.pos_x};
//     double dy {tf2.pos_y-tf1.pos_y};
//     double dr2 {pow(dx, 2) + pow(dy, 2)};
//     double D {tf1.pos_x*tf2.pos_y - tf2.pos_x*tf1.pos_y};
    
//     double discriminant {pow(lookaheadDistance, 2)*dr2 - pow(D, 2)};

//     Transform returnTf {};

//     if (discriminant >= 0)
//     {
//         // One or two intersections
//         Transform sol1 {};
//         sol1.pos_x = (D*dy+copysignl(1.0, dy)*dx*sqrt(discriminant))/(dr2);
//         sol1.pos_y = (-D*dx+abs(dy)*sqrt(discriminant))/(dr2);
//         sol1.rot_z = tf2.rot_z;

//         Transform sol2 {};
//         sol2.pos_x = (D*dy-copysignl(1.0, dy)*dx*sqrt(discriminant))/(dr2);
//         sol2.pos_y = (-D*dx-abs(dy)*sqrt(discriminant))/(dr2);
//         sol2.rot_z = tf2.rot_z;

//         // Select the point that is is furthest along the line
//         Transform diffTf = sol2-sol1;
//         if (diffTf.pos_y > 0)
//         {
//             returnTf = sol2;
//         }
//         else
//         {
//             returnTf = sol1;
//         }

//         // Set the returnTf to the last known frame
//         // Realtive to the robot
//         CoordinateFrame lastKnown {&(globComm->poseComm.robotFrame), returnTf};
//         // Transform to the parent of the lastknownframe
//         lastKnownFrame.transform = lastKnown.getTransformLevelTo(lastKnownFrame.getParent(), 2, 2);

//     }
//     else
//     {
//         // No intersections
        
//         if (lastKnownFrame.transform == Transform{})
//         {
//             // If last known frame does not exist
//             returnTf = tf2;
            
//         }
//         else
//         {
//             // If last known frame does exist
//             returnTf = lastKnownFrame.getTransformLevelTo(globComm->poseComm.robotFrame.getParent(), 2, 2);
//         }


//     }

//     return returnTf;

// }


// void PathFollower::runLoop()
// {
//     // If the current target has been visited
//     // if (pathFrameVisited(&(path.keyFrames.at(pathSegmentIndex+1))))
//     // {
//     //     // Go to the next target point
//     //     ++pathSegmentIndex;
//     // }

//     // Calculate the wanted turn speed
//     double turnSpeed {getRotSpeedDriving()};

//     // Set the turnspeed to the kinematic driver and in turn the motor driver.
//     #warning translational speed not set
//     driver.calcSpeeds(0, turnSpeed);
//     driver.setSpeeds();
// }

// CoordinateFrame PathFollower::getLookaheadCFInterpolated()
// {
    // Sort the coordinateFrames in the interpolatedframes from closest to furthest away from the lookaheadDistance.

    // Then pick the two closest ones. These are the intersections points.

    // Determine which one is forwards, but how?
    // Ideas: Choose the one closest to the next keyframe? But then I would need to keep track of which keyframe I am on and how to detect a change. Maybe if you have come close enough it counts as passing?

    // Special handling of when you get to the end of the path? (When the last interpolatedpoint on the whole path is closer than the lookaheadDistance)
// }


// bool PathFollower::pathFrameVisited(PathFrame* pFrame)
// {
//     // Get the robot pose in the coordinate system of the visiting tile
//     Transform resultTf {globComm->poseComm.robotFrame.getTransformLevelTo(&(pFrame->frame), 1, 2)};
//     // If you are past the line that is visitedradius from the point to the previous point
//     if (resultTf.pos_y > -pFrame->visitedRadius)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }