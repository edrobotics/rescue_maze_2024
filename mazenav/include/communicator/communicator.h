#pragma once
#include <mutex>
#include <iostream>
#include <chrono>
#include <array>

#include "GlobalConstants.h"
#include "transformations/tfsys.h"
#include "fusion/MotorControllers.h"
#include "communicator/logger.h"
#include "communicator/timer.h"
#include "communicator/navigationCommunicator.h"
#include "communicator/tileDrivePropertyCommunicator.h"
#include "communicator/panicFlagCommunicator.h"
#include "communicator/victimDataCommunicator.h"

namespace communication
{
    class PoseCommunicator
    {
        private:
            // std::mutex mtx_world;
            // std::mutex mtx_localtile;
            // std::mutex mtx_robot;

            // For timed average of robotSpeed
            static constexpr int HISTORY_NUM {4};
            int historyIndex {0};
            void incrementHistoryIndex();
            void calcAverage(CoordinateFrame& result);

            // To enable initialization with constructor?
            template <typename T, std::size_t ... Is>
            constexpr std::array<T, sizeof...(Is)>
            create_array(T value, std::index_sequence<Is...>)
            {
                // cast Is to void to remove the warning: unused value
                return {{(static_cast<void>(Is), value)...}};
            }
            template <std::size_t N, typename T>
            constexpr std::array<T, N> create_array(const T& value)
            {
                return create_array(value, std::make_index_sequence<N>());
            }

            std::array<CoordinateFrame, HISTORY_NUM> robotSpeeds {create_array<HISTORY_NUM, CoordinateFrame>(CoordinateFrame{nullptr})};


            // Used for sensor data filtering
            bool isTurning {false};
            bool isDriving {false};
            std::mutex mtx_controlVars {};

            double frontObstacleDist {-1};

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
            // CoordinateFrame robotSpeed {nullptr};
            CoordinateFrame robotSpeedAvg {nullptr};
            CoordinateFrame robotSpeedCur {nullptr};
            // Take in a new speed and calculate robotSpeedAvg based on this.
            void calcRobotSpeedAvg(CoordinateFrame newSpeed);

            // Indicates whether or not values have been set before
            int freshness {6};

            // Indicates whether new values have been set. Set to true when putting in new values (PoseEstimator), false when reading (PathFollower)
            bool updated {false};
            
            // Time point for when the lastRobotTime was captured
            std::chrono::steady_clock::time_point curRobotTime {std::chrono::steady_clock::now()};
            std::chrono::steady_clock::time_point lastRobotTime {std::chrono::steady_clock::now()};
        
            std::mutex mtx_general {};
            // Used by PathFollower. Is here to ensure synchronisation
            // DO NOT USE DIRECTLY! ACCESS THROUGH THREAD SAFE FUNCTIONS
            CoordinateFrame targetFrame {&localTileFrame};


            void setTurning(bool turning);
            bool getTurning();
            void setDriving(bool driving);
            bool getDriving();


            void setFrontObstacleDist(double dist);
            double getFrontObstacleDist();


            CoordinateFrame startLocalTileFrame {&worldFrame};
            // Returns true if the robot is not on the same tile as startLocalTileFrame
            bool hasDrivenStep();

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
        TileDrivePropertyCommunicator tileInfoComm;
        PanicFlagCommunicator panicFlagComm;
        VictimDataCommunicator victimDataComm{};
        Logger logger {};
        Timer timer {};
    };
}