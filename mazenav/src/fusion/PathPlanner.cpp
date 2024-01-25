#include "localNav/PathPlanner.h"


void PathPlanner::setPath(Path path)
{
    this->path = path;
}

void PathPlanner::setGlobalPath(std::vector<communication::DriveCommand> globalPath)
{
    this->globalPath = globalPath;
}


void PathPlanner::calculate(RobotPose startPose)
{
    // Add initial keypose
    path.addKeyPose(startPose);
    
    // Add the remaining keyposes
    RobotPose curPose {startPose};
    for (communication::DriveCommand move : globalPath)
    {
        #warning Will not work currently. Need to figure out transforms
        switch (move)
        {
            case communication::driveForward:
                curPose.t_y += GRID_SIZE;
                break;
            case communication::turnLeft:
                curPose.r_z += 90;
                break;
            case communication::turnRight:
                curPose.r_z -= 90;
                break;
            default:
                break;
        }

        path.addKeyPose(curPose);
    }

    path.interpolate();
}