#pragma once
#include <queue>

#include "GlobalConstants.h"
#include "localNav/Path.h"
#include "communicator/communicator.h" // For the drivecommands

// Top-level class to plan a path to the next tile
// Probably needed subclasses:
// Path
// Interpolator

class PathPlanner
{
    public:
        void setPath(Path path);
        Path path {};

        void setGlobalPath(std::vector<communication::DriveCommand> globalPath);
        std::vector<communication::DriveCommand> globalPath {};

        void calculate(RobotPose startPose);


    private:

};