// This file contains the class that interprets sensor values into the robot pose.

// TODO:
// Write the getter and checker functions for the values and contributions
// Handle value wrapping
// Minimize ToF calculations - how?
// Check the warnings
// Testing - includes writing test function?
// More advanced fusing

#pragma once

#include <thread>
#include <iostream>

#include "fusion/Sensors.h"
#include "fusion/TeensyCommunicator.h"
#include "communicator/communicator.h"


class PoseEstimator
{
    public:
        PoseEstimator(communication::Communicator* globComm, TeensyCommunicator* tComm);

        // Spawns a new thread running everything that needs to be run
        void begin();

        // Stops the thread spawned by begin
        void stop();

        // Test the module. Returns true if all tests pass.
        bool test();

        // Describes different fusion groups (combinations of raw data to base pose estimate on)
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
        TeensyCommunicator* tComm;
        // Sensor objects
        #warning if sensors in this object are used elsewhere, concurrency issues might arise
        Sensors sensors;


        // Fusion group in use
        FusionGroup fusionGroup {fg_none};

        // The thread that updates.
        // Not in a state of execution by default (https://en.cppreference.com/w/cpp/thread/thread)
        std::thread updater{};


        // High-level under-the-hood functions
        // The loop to run in the new thread. Conatins the update with correct delays and such.
        void runLoop();
        // Whether to stop the thread or not. Used to terminate it.
        bool stopThread {false};
        // Loops runLoop() as long as stopThread==false. Simplifies as I do not have to figure out lambdas now, and I know that this works
        void runLoopLooper();
        // Update the pose with the supplied FusionGroup
        void update(FusionGroup fgroup);


        // High-level pose estimation functions. Return the estimated pose.
        // Simple fusiongroup
        communication::PoseCommunicator updateSimple();
        // Lidar fusiongroup
        communication::PoseCommunicator updateLidar();
        // Lidar+simple fusiongroup
        communication::PoseCommunicator updateLidarSimple();
        // Only gives rotation, not translations
        communication::PoseCommunicator updateIMU();




        // Low-level pose estimation helper functions
        // The distance travelled (diff) since last check or reset
        double getWheelTransDiff();

        // Estimated rotation diff since last check or reset
        double getWheelRotDiff();

        // Distance travelled since last check or reset
        #warning Unclear what distance diff I get from ToF. Should be straight out from the sensor, which this function assumes
        double getTofTransDiff();

        // Get the absolute Y translation measured by ToF. Not sure if it will work.
        double getTofYTrans();

        // Get the absolute X translation measured by ToF
        double getTofXTrans();

        // Get the absolute Z rotation measured by ToF
        double getTofZRot();

        // Get the Z rotation diff since last check or reset
        double getIMURotDiff();


        // Check usability of values:
        // For X position and Z rotation
        bool getIsTofXAbsolute();
        // For Y position
        bool getIsTofYAbsolute();
        // If ToF values are usable for even diff
        bool getIsTofDiff();

        void wrapPoseComm(communication::PoseCommunicator& posecomm);
        const int minYPos {0};
        const int maxYPos {300};
        const int minXPos {0};
        const int maxXPos {300};
        const double minZRot {-M_PI};
        const double maxZRot {M_PI};
        const double tileRotThreshhold {M_PI_4}; // 45deg


};