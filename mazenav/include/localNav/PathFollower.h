// This is a class that takes in a path and robot position, then outputs chassis speeds to follow the path.
// The algorithm used is the pure pursuit algorithm.

// Usage:
// Set the path to follow.
// Run the runloop at regular intervals (every iteration)


// Left to do:
// Bounds checking for the path segment index everywhere

#pragma once

#include <iostream>
#include <fstream>
#include <string>

#include "localNav/KinematicDriver.h"
#include "communicator/communicator.h"
#include "localNav/Path.h"
#include "localNav/PIDController.h"

class PathFollower
{
    public:
        PathFollower(communication::Communicator* globComm);

        ~PathFollower();

        // Sets the path to follow
        // void setPath(Path path);

        // Calculates the wanted speeds, sets them to KinematicDriver applies KinematicDriver's motor speeds to the motor controller
        void runLoop();

        // Runs the runLoop forever
        void runLoopLooper();

        // Set the pos_y to follow.
        void setLinePos(double newYLine);


    private:
        communication::Communicator* globComm;
        KinematicDriver driver;

        // Input y error, get out wanted angle (correction)
        PIDController yPid {0.1, 0, 0};
        // Input angle error, output wanted chassis speeds correction
        PIDController angPid {10, 0, 0};

        // Do the driving in accordance with parameters set earlier
        void drive(int direction);
        // Turn in accordance with parameters set earlier
        void turn(int direction);


        // Set the target point given a drivecommand
        void setTargetPointTf(communication::DriveCommand dC);
        // The target point
        // CoordinateFrame targetPoint {nullptr};
        // The transform from targetpoint to robotFrame (targetpoint in relation to robotframe)
        // Transform targetPointTf {};
        // Distance left to the target
        double distLeftToTarget {};
        // Calculate the distance mentioned above
        double getDistLeftToTarget();
        // Angle left to the target
        double angLeftToTarget {};
        // Calculate the angle mentioned above. Gives the shortest possible solution.
        double getAngLeftToTarget();

        // Checks if the robot has completed the move by looking at the position of the robot.
        // direction - (+1) or (-1), where the sign indicates the desired direction of movement.
        bool checkIsFinishedDriving(int direction);
        bool checkIsFinishedTurning(int direction);

        static constexpr double DRIVE_STOP_THRESHOLD {20};
        static constexpr double TURN_STOP_THRESHOLD {0.08};




        // Old pid
        // PIDController pid {1, 0, 0};

        // int pathSegmentIndex {0};
        // Path path;
        // CoordinateFrame lookaheadCf {nullptr};
        // CoordinateFrame lastKnownFrame {nullptr};

        // Pure pursuit parameters
        // #warning untuned parameters
        // double lookaheadDistance {0}; // Can (should) change depending on the speed? Should probably limit on both ends.

        // Lookahead limits
        // const double minLookaheadDistance {10};
        // const double maxLookaheadDistance {69};
        // Set the lookahead distance. If set out of limits, sets to closest limit and returns false
        // bool setLookaheadDistance(double distance);

        // Functions to go through the steps

        // <something more here>
        // Returns a childless copy of the CF in the path that is closest to the wanted lookaheadpoint.
        // CoordinateFrame getLookaheadCFKeyframe();
        // CoordinateFrame getLookaheadCFInterpolated();
        // Get the transform from the robot to the lookaheadPoint
        // Transform getLookaheadTF();
        // Get the direction of the lookaheadpoint. It is the angle between the connecting line and the y-axis of the robot.
        // double getLookaheadAngle();

        // Return if the given PathFrame has been visited
        // bool pathFrameVisited(PathFrame* pFrame);

        // Calculate the wanted rotational speed of the robot for driving
        double getRotSpeedDriving();
        // Calculate the wanted translational speed of the robot for driving
        double getTransSpeedDriving();

        double getRotSpeedTurning();
        double getTransSpeedTurning();


        // The threshold for distanceLeft, for when to use PID for translational speed control.
        static constexpr double DRIVING_CLOSE_PID_THRESHOLD {69};
        // The threshold for angleLeft, for when to use PID for rotational speed control.
        static constexpr double TURNING_CLOSE_PID_THRESHOLD {0.35}; // About 20 degrees

        #warning untuned constants
        PIDController driveTransSpeedPid {2, 0, 0};
        PIDController turnRotSpeedPid {1, 0, 0};
        PIDController turnTransSpeedPid {1, 0, 0};


        static constexpr double DRIVE_SPEED_STANDARD {200};
        static constexpr double DRIVE_SPEED_SLOW {50};
        static constexpr double TURN_SPEED_STANDARD {M_PI_2};
        static constexpr double TURN_SPEED_SLOW {M_PI_4};


        // For getting PID values from file
        void readPidFromFile(double& kP, double& kI, double& kD);

};