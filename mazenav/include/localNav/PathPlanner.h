#pragma once
#include <queue>

#include "communicator/communicator.h" // For the drivecommands
#include "GlobalConstants.h"
#include "localNav/Path.h" // For paths
#include "transformations/tfsys.h" // Everything transform-related

// Top-level class to plan a path to the next tile
// Probably needed subclasses:
// Path
// Interpolator

class PathPlanner
{
    public:
        // Set a path to follow
        void setPath(Path path);

        // Sets the globalPath. The globalPath is an ordered collection of DriveCommands (eg. forward, turn right, turn left etc.)
        void setGlobalPath(std::vector<communication::DriveCommand> globalPath);

        // Calculate the path from startPose along the globalPath
        void calculate();


    private:
        // The path that will be followed
        CoordinateFrame startFrame {nullptr};
        Path path {};
        // The "path" of global commands
        std::vector<communication::DriveCommand> globalPath {};

        const Transform tfForward {0, GRID_SIZE, 0, 0, 0, 0};
        const Transform tfRight {0, 0, 0, 0, 0, -M_PI_2};
        const Transform tfLeft {0, 0, 0, 0, 0, M_PI_2};

        // Fills the keyframes.
        // The resulting frames have the sFrame as their parent and are relative it.
        void fillKeyFrames(CoordinateFrame sFrame, std::vector<communication::DriveCommand>& globPath);

};