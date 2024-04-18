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
#include "fusion/LedControl.h"

class PathFollower
{
    public:
        PathFollower(communication::Communicator* globComm, PiAbstractor* pAbs);

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
        PiAbstractor* piAbs {};
        LedControl ledController {};

        // Input y error, get out wanted angle (correction)
        PIDController yPid {0.008, 0, 0.0017};
        // Caps a number within limits
        #warning untuned constant
        static constexpr double YPID_MAX_MIN_OUTPUT_ANGLE {M_PI_4*0.8};
        double capYPidOutput(double output);
        // Input angle error, output wanted chassis speeds correction
        PIDController angPid {2, 0, 0.01};

        // Do the driving in accordance with parameters set earlier
        void drive();
        // Turn in accordance with parameters set earlier
        void turn();
        // Align angle
        // usePidAng - if false, target angle is 0. If true, target angle is determined by what the driving PID wants for the given parameters.
        void alignAngle(bool usePidAng);


        // Set the target point given a drivecommand
        void setTargetPointTf(communication::DriveCommand dC);
        // Set target point manually with a transform (for alignment)
        void setTargetPointTf(Transform tf);
        // Set the targetPoint to return to the tile which you started from.
        void setBackWardTargetPointTf();
        // Get turning direction given current robot pose and target robot pose
        int getTurnDirection();
        // Get driving direction given current robot pose and target robot pose
        int getDriveDirection();
        
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
        // If true, abort the current move.
        bool abortMove {};

        static constexpr double DRIVE_STOP_THRESHOLD {20};
        static constexpr double TURN_STOP_THRESHOLD {M_PI_4/3.0};
        static constexpr double FRONT_OBSTACLE_DRIVE_STOP_THRESHOLD {80};
        static constexpr double FRONT_OBSTACLE_STANDING_STOP_THRESHOLD {20};




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
        double getRotSpeedDriving(int direction);
        // Calculate the wanted translational speed of the robot for driving
        double getTransSpeedDriving(int direction);

        double getRotSpeedTurning(int direction);
        double getTransSpeedTurning();


        // The threshold for distanceLeft, for when to use PID for translational speed control.
        static constexpr double DRIVING_CLOSE_PID_THRESHOLD {100}; // 100 is good
        // The threshold for angleLeft, for when to use PID for rotational speed control.
        static constexpr double TURNING_CLOSE_PID_THRESHOLD {M_PI_4}; // M_PI_4

        // Tuned
        PIDController driveTransSpeedPid {0.069, 0, 0};
        // Untuned constant
        PIDController turnTransSpeedPid {1, 0, 0};
        // Tuned
        PIDController turnRotSpeedPid {0.1, 0, 0};


        static constexpr double DRIVE_SPEED_STANDARD {200};
        static constexpr double DRIVE_SPEED_SLOW {100};
        static constexpr double TURN_SPEED_STANDARD {M_PI_2*0.75};
        static constexpr double TURN_SPEED_SLOW {TURN_SPEED_STANDARD};


        // For getting PID values from file
        void readPidFromFile();


        // Panic handling
        void checkAndHandlePanic();
        void handleVictim();
        void handleBlackTile();
        bool driveBackwardsBlacktile {false};
        void handleLOP();


};