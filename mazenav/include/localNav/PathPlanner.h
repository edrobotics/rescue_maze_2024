#pragma once
#include <queue>

#include "communicator/communicator.h" // For the drivecommands
#include "GlobalConstants.h"
#include "localNav/Path.h" // For paths
#include "transformations/tfsys.h" // Everything transform-related

// Usage:
// Set a start frame and a global path.
// Calculate the path.
// Get the path with getPath (and feed it into pathFollower)

class PathPlanner
{
    public:
        // Set a path to follow - Why would this be needed?
        // void setPath(Path path);

        // Set the frame that the robot should start on.
        void setStartFrame(CoordinateFrame frame);

        // Sets the globalPath. The globalPath is an ordered collection of DriveCommands (eg. forward, turn right, turn left etc.)
        void setGlobalPath(std::vector<communication::DriveCommand> globalPath);

        // Calculate the path from startPose along the globalPath
        void calculate();

        // Returns the generated path
        Path getPath();


    private:
        // The frame to start from
        CoordinateFrame startFrame {nullptr};
        // The path that will be followed
        Path path {};
        // The "path" of global commands
        std::vector<communication::DriveCommand> globalPath {};

        // Go forward one step
        const Transform tfForward {0, GRID_SIZE, 0, 0, 0, 0};
        // Turn right
        const Transform tfRight {0, 0, 0, 0, 0, -M_PI_2};
        // Turn left
        const Transform tfLeft {0, 0, 0, 0, 0, M_PI_2};

        // Fills the keyframes.
        // The resulting frames have the first keypose as their parent and are relative it.
        void fillKeyFrames(std::vector<communication::DriveCommand>& globPath);

};