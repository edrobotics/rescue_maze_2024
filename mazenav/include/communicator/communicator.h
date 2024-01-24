#pragma once
#include <queue>
#include <mutex>
#include "communicator/RobotPose.h"

namespace communication
{
    // Possible drive commands that can be issued from global navigation to local navigation
    enum DriveCommand
    {
        driveForward,
        turnLeft,
        turnRight,
        turnBack,
        noAction,
        driveCommand_num,
    };

    // Methods for interacting with the communication
    class Navigation //Rename? - in general also
    {
        public:
        // Pushes a command to the command queue
        void pushCommand(DriveCommand command);
        // Gets the next command from the command queue
        // pop - false if it should not modify the queue, true if the first element should be removed
        DriveCommand getCommand(bool pop);

        private:
        std::mutex commands_mutex;
        std::queue<DriveCommand> commands;
    };

    class PoseCommunicator
    {
        public:
            // Sets the robot pose
            void setPose(RobotPose pose);

            // Gets the robot pose
            RobotPose getPose();
        
        private:
            RobotPose pose {};
            std::mutex mtx_pose;
    };

    // Class containing the data that we want to share
    class Communicator
    {
        public:
        Navigation navigationComm;
        PoseCommunicator poseComm {};
    };
}