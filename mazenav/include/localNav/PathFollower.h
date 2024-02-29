// This is a class that takes in a path and robot position, then outputs chassis speeds to follow the path.
// The algorithm used is the pure pursuit algorithm.

// Usage:
// Set the path to follow.
// Run the runloop at regular intervals (every iteration)


// Left to do:
// Bounds checking for the path segment index everywhere

#pragma once

#include "localNav/KinematicDriver.h"
#include "communicator/communicator.h"
#include "localNav/Path.h"
#include "localNav/PIDController.h"

class PathFollower
{
    public:
        PathFollower(communication::Communicator* globComm);

        // Sets the path to follow
        void setPath(Path path);

        // Calculates the wanted speeds, sets them to KinematicDriver applies KinematicDriver's motor speeds to the motor controller
        void runLoop();

    private:
        communication::Communicator* globComm;
        KinematicDriver driver;
        PIDController pid {1, 0, 0};

        int pathSegmentIndex {0};
        Path path;
        CoordinateFrame lookaheadCf {nullptr};
        CoordinateFrame lastKnownFrame {nullptr};

        // Pure pursuit parameters
        #warning untuned parameters
        double lookaheadDistance {0}; // Can (should) change depending on the speed? Should probably limit on both ends.

        // Lookahead limits
        const double minLookaheadDistance {10};
        const double maxLookaheadDistance {69};
        // Set the lookahead distance. If set out of limits, sets to closest limit and returns false
        bool setLookaheadDistance(double distance);

        // Functions to go through the steps

        // <something more here>
        // Returns a childless copy of the CF in the path that is closest to the wanted lookaheadpoint.
        // CoordinateFrame getLookaheadCFKeyframe();
        // CoordinateFrame getLookaheadCFInterpolated();
        // Get the transform from the robot to the lookaheadPoint
        Transform getLookaheadTF();
        // Get the direction of the lookaheadpoint. It is the angle between the connecting line and the y-axis of the robot.
        double getLookaheadAngle();

        // Return if the given PathFrame has been visited
        bool pathFrameVisited(PathFrame* pFrame);

        // Calculate the wanted turning speed of the robot. Does everything for you.
        double getTurnSpeed();

};