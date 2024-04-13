#pragma once
#include <queue>
#include <mutex>
#include <iostream>
#include <chrono>

#include "transformations/tfsys.h"
#include "fusion/MotorControllers.h"
#include "communicator/logger.h"

namespace communication
{
    // Possible drive commands that can be issued from global navigation to local navigation
    enum class DriveCommand
    {
        driveForward,
        turnLeft,
        turnRight,
        turnBack,
        noAction,
        driveCommand_num,
    };

    // Methods for interacting with the communication
    class NavigationCommunicator
    {
        public:
        // Pushes a command to the command queue
        void pushCommand(DriveCommand command);
        // Gets the next command from the command queue and removes it from the queue
        DriveCommand popCommand();

        private:
        std::mutex mtx_commands {};
        std::queue<DriveCommand> commands {};
    };

    class PoseCommunicator
    {
        public:
            // Default constructor
            PoseCommunicator();
            // Copy constructor
            PoseCommunicator(const PoseCommunicator& pComm);

            // Copy the whole structure of frames.
            PoseCommunicator& operator=(const PoseCommunicator& pComm);

            // Thread safety is handled inside of the CoordinateFrame class
            CoordinateFrame worldFrame {nullptr};
            CoordinateFrame localTileFrame {&worldFrame};
            CoordinateFrame robotFrame {&localTileFrame};
            CoordinateFrame lastRobotFrame {&localTileFrame};

            void setTargetFrameTS(CoordinateFrame& frame);
            CoordinateFrame getTargetFrame();
            void setTargetFrameTransformTS(Transform tf);
            
            // Speeds can be represented with coordinateframes. The speeds are relative to the parent object. (unsure if this representation is actually okay)
            // CoordinateFrame worldSpeed {nullptr};
            // CoordinateFrame localTileSpeed {&worldSpeed};
            CoordinateFrame robotSpeed {nullptr};

            // Indicates whether or not values have been set before
            int freshness {6};
            
            // Time point for when the lastRobotTime was captured
            std::chrono::steady_clock::time_point curRobotTime {std::chrono::steady_clock::now()};
            std::chrono::steady_clock::time_point lastRobotTime {std::chrono::steady_clock::now()};
        
            std::mutex mtx_general {};
        private:
            // Used by PathFollower. Is here to ensure synchronisation
            CoordinateFrame targetFrame {&localTileFrame};
            // std::mutex mtx_world;
            // std::mutex mtx_localtile;
            // std::mutex mtx_robot;

    };

    class MotorControllerCommunicator
    {
        public:
            // Sets the speeds that should be set to the motors
            void setSpeeds(MotorControllers::MotorSpeeds speeds);

            // Reads the speeds that should be set to the motors
            MotorControllers::MotorSpeeds getSpeeds();

        private:
            MotorControllers::MotorSpeeds speeds {};
            std::mutex mtx_speeds;

    };

    // Class containing the data that we want to share
    class Communicator
    {
        public:
        NavigationCommunicator navigationComm;
        PoseCommunicator poseComm {};
        MotorControllerCommunicator motors{};
        Logger logger {};
    };
}