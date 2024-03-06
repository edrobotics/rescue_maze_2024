// This file contains the class that interprets sensor values into the robot pose.

// TODO:
// Implement the actual fusion. Start simple
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
        communication::PoseCommunicator updateSimple();
        communication::PoseCommunicator updateLidar();
        communication::PoseCommunicator updateLidarSimple();
        // Only gives rotation, not translations
        communication::PoseCommunicator updateIMU();




        // Low-level pose estimation helper functions

};