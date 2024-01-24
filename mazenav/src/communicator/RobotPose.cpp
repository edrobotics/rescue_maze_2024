#include "communicator/RobotPose.h"


double RobotPose::calcDistance2d(RobotPose pose1, RobotPose pose2)
{
    return sqrt(powf64(pose1.t_x-pose2.t_x, 2) + powf64(pose1.t_y-pose2.t_y, 2));
}