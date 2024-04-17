// This file contains the class that interprets sensor values into the robot pose.

// TODO:
// Check the warnings
// For the first iterations, values might be weird. Initiate all "last values" when initializing object?
// Testing - includes writing test function?
// Minimize ToF calculations - how?
// More advanced fusing


// Note before stopping today:
// Should the fusing be like it is, where we only use the best method available, or should everything be fused all the time?
// That would require a structure change where we first check all contributions (if they are valid), then do a calculation with them.
// Currently, the things relating to tof y diff and wheel diff are under (re)construction, which means that updateSimple is also under reconstruction.

#pragma once

#include <thread>
#include <iostream>

#include "fusion/Sensors.h"
#include "fusion/TeensyCommunicator.h"
#include "fusion/Average.h"
#include "communicator/communicator.h"
#include "GlobalConstants.h"


class PoseEstimator
{
    public:
        PoseEstimator(Sensors* sens);

        // Spawns a new thread running everything that needs to be run
        // void begin();
        // Loops runLoop() as long as stopThread==false. Simplifies as I do not have to figure out lambdas now, and I know that this works
        void runLoopLooper(communication::Communicator* globComm);

        // Stops the thread spawned by begin
        // void stop();

        // Test the module. Returns true if all tests pass.
        bool test();

        // Describes different fusion groups (combinations of raw data to base pose estimate on)
        // Maybe the binary stuff is not the best idea...
        enum FusionGroup
        {
            fg_none         = 0,
            fg_simple       = 0b00000001,
            fg_lidar        = 0b00000010,
            fg_imu          = 0b00000100,
            fg_lidar_simple = fg_lidar | fg_simple,
            fg_lidar_imu    = fg_lidar | fg_imu,
        };

        // Sets the fusion group to base the pose off of
        void setFusionGroup(FusionGroup fgroup);




    private:

        // For external data/communication:
        // Global communicator
        communication::Communicator* globComm;
        // Teensy communicator
        // TeensyCommunicator* tComm;
        // Sensor objects
        Sensors* sensors;


        // Fusion group in use
        FusionGroup fusionGroup {fg_none};
        // Whether currently turning or not - influences certain thresholds and what values may be used.
        bool isTurning {false};
        bool isDriving {false};

        // The thread that updates.
        // Not in a state of execution by default (https://en.cppreference.com/w/cpp/thread/thread)
        std::thread updater{};


        // High-level under-the-hood functions
        // The loop to run in the new thread. Conatins the update with correct delays and such.
        void runLoop();
        // Whether to stop the thread or not. Used to terminate it.
        bool stopThread {false};
        // Update the pose with the supplied FusionGroup
        void update(FusionGroup fgroup, bool doUpdate);
        // Run the calculation without actually updating. Does it 5 times.
        void flush(FusionGroup fgroup);


        // High-level pose estimation functions. Return the estimated pose.
        // Simple fusiongroup
        communication::PoseCommunicator updateSimple();
        // Lidar fusiongroup
        communication::PoseCommunicator updateLidar();
        // Lidar+simple fusiongroup
        communication::PoseCommunicator updateLidarSimple();
        // Only gives rotation, not translations
        communication::PoseCommunicator updateIMU();

        // Calculate and set the speeds
        void calcSpeeds(communication::PoseCommunicator& pose);


        // Low-level pose estimation helper functions
        // Calculate the difference in wheel distance compared to last iteration
        void calcWheelDistanceDiffs();
        // Current motor distance diffs
        MotorControllers::Distances motorDistanceDiffs{};
        // Last absolute position for wheels
        MotorControllers::Distances lastMotorDistances {};
        // The distance travelled (diff) since last check or reset
        ConditionalAverageTerm getWheelTransDiff();
        // Estimated rotation diff since last check or reset
        ConditionalAverageTerm getWheelRotDiff();

        // Distance travelled since last check or reset
        ConditionalAverageTerm getTofTransYDiff();
        double lastTofFR {};
        double lastTofFL {};
        double lastTofB {};

        // Get the absolute Y translation measured by ToF. Not sure if it will work.
        // angle - current robot angle
        // yoffset - robot centre to back/front sensors, Y direction
        // xoffset - robot centre to front sensor, X direction
        ConditionalAverageTerm getTofYTrans(double angle, double yoffset, double xoffset);

        // Get the absolute Z rotation measured by ToF
        ConditionalAverageTerm getTofZRot(double curAng);

        // Get the absolute X translation measured by ToF
        ConditionalAverageTerm getTofXTrans(double angle);

        // Helpers for Tof XTrans and ZRot
        double getAngleFromTwoTof(double d1, double d2, double yoffset);
        double getCentredistanceFromTwoTof(double d1, double d2, double centreoffset, double angle);

        // Get the Z rotation diff since last check or reset
        ConditionalAverageTerm getIMURotDiff();
        double lastImuAngle {};
        double lastImuWasReset {false};


        /////////////////////////
        // Sensor value limits //

        // Maximum allowed wheel spin between iterations
        static constexpr int MAX_WHEEL_ODOM_DIFF {50};
        // Maximum diff for IMU angle values between two iterations. Based on max speed of two rounds per second for robot
        static constexpr double MAX_IMU_Z_ROT_DIFF {1.25};
        // Maximum diff for IMU angle that is considered frozen in place (frozen without reset), while wheels are also moving
        static constexpr double MAX_IMU_FROZEN_ANGLE {0.01}; // Approx. 0.6 degrees
        // Minimum diff between IMU and wheelRotation for the IMU to be considered frozen
        static constexpr double MIN_IMU_WHEEL_DIFF_FROZEN {0.02}; // Approx. 3 degrees
        // Maximum diff for ToF sensor values between two measurements
        static constexpr int MAX_TOF_Y_DIFF {50};
        // Maximum angle for ToF to be used for diff calc
        static constexpr double MAX_Z_ROTATION_Y_TOF_DIFF {M_PI_4/5.0};
        // Maximum difference between front ToF sensor values (to prevent side walls from messing it up).
        static constexpr double MAX_TOF_Y_ABS_DIFF {20};
        // Maximum angle for ToF to be used for abs calc y direction
        static constexpr double MAX_Z_ROTATION_Y_TOF_ABS {MAX_Z_ROTATION_Y_TOF_DIFF};
        // Maximum angle for ToF X trans and Z rot calculation
        static constexpr double MAX_ZROT_XTRANS_TOF_ABS {M_PI_4/2};
        // Maximum angle for ToF X trans and Z rot calculation while turning
        static constexpr double MAX_ZROT_XTRANS_TOF_ABS_TURNING {M_PI_4/4};
        // Maximum distance that can be used for abs calc with Y Tof
        static constexpr double MAX_TOF_Y_DIST_ABS {600};
        // If robot centre to wall/calculated sensor measurement is > this, wall is not deemed present
        #warning untuned variable
        static constexpr double WALL_PRESENCE_THRESHOLD_CENTRE {220};
        // If sensor value above this, wall deemed not present
        #warning untuned variable
        static constexpr double WALL_PRESENCE_THRESHOLD_SENSOR {140};

        // Can TOF in robot X direction (left) be used for absolute positioning?
        bool getIsTofXLeft();
        // Can TOF in robot X direction (right) be used for absolute positioning?
        bool getIsTofXRight();

        // // Check usability of values:
        // // Can TOF pointing in robot X direction be used for absolute positioning? (either right, left or both)
        // bool getIsTofXAbsolute();
        // // Can TOF in robot Y direction be used for absolute positioning?
        // bool getIsTofYAbsolute();
        // // If ToF in robot Y direction are usable for differential calculations
        // // Returns 8-bit int with the three last bits meaning: (1 means OK, 0 means not OK)
        // #define Y_TRANS_DIFF_FL 0b00000001
        // #define Y_TRANS_DIFF_FR 0b00000010
        // #define Y_TRANS_DIFF_B  0b00000100
        // uint8_t getIsTofYDiff();
        // uint8_t getIsWheelTransDiff();

        // Sensor value limits end //
        /////////////////////////////

        // Wraps the value between lower and upper.
        // Currently using loops. Can probably be optimised.
        #warning maybe only works for integer bounds?
        #warning check behaviour at endpoints. Should go up to, not including upper.
        double wrapValue(double value, double lower, double upper);
        // Wrap the two values so that they are within the same span (instead of 299 and 1 you would have 299 and 301)
        // If one value must be outside, the upper limit is always preferred
        // void wrapValueSameScale(double& val1, double& val2, double lower, double upper);
        // Wraps all the elements of a vector into one scale, like wrapValueSameScale does.
        void wrapVectorSameScale(std::vector<ConditionalAverageTerm>& vec, double lower, double upper);

        // Wrap the pose values into the current tile. Does not register tile changes, only bounds the values
        void wrapPoseComm(communication::PoseCommunicator& posecomm);
        // Given a poseComm and the last poseComm, ghost-move the local tile to register tile transitions
        // Assumes that input posecomm has values inside of limits.
        // Will only register one "step" - for example cannot register two tile moves forward in one execution, as that is not feasible with these speeds and loop times
        void updatePoseComm(communication::PoseCommunicator& pose);
        // Limits for wrapping a pose
        const int minYPos {0};
        const int maxYPos {GRID_SIZE};
        const int minXPos {0};
        const int maxXPos {GRID_SIZE};
        const double tileRotThreshhold {M_PI_4}; // 45deg
        const double minZRot {-tileRotThreshhold};
        const double maxZRot {tileRotThreshhold};
        // If rot diff > this, then tile change has occured
        const double tileRotDiffThreshold {M_PI_4};
        // If x trans diff > this, then tile change has occured
        const double tileTransXDiffThreshold {GRID_SIZE/2.0};
        // If y trans diff > this, then tile change has occured
        const double tileTransYDiffThreshold {GRID_SIZE/2.0};
        // Transforms to move the tile to wrap the pose
        Transform tf_rotateTileRight {0, GRID_SIZE, 0, 0, 0, -M_PI_2};
        Transform tf_rotateTileLeft {GRID_SIZE, 0, 0, 0, 0, M_PI_2};

        // Get the wall presence. This is relative to the tile.
        bool getLeftWallPresent();
        bool getRightWallPresent();
        bool getFrontWallPresent();
        bool getBackWallPresent();

        // Returns true if there is either a wall or something else in front of the robot.
        // Does not update sensor values - works with those that already exist
        double getFrontObstacleDist(Tof::TofData td);
        // Front sensor value where we think something is there.
        #warning untuned constant
        static constexpr int FRONT_OBSTACLE_DETECTION_THRESHOLD {100};

};