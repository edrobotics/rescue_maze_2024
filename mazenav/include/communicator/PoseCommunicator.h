#pragma once

#include <mutex>
#include <iostream>
#include <chrono>
#include <array>
#include <thread>

#include "GlobalConstants.h"
#include "transformations/tfsys.h"
#include "communicator/PoseDataSyncBlob.h"


namespace communication
{
    class PoseCommunicator
    {
        private:

            PoseDataSyncBlob poseDataBlob {};
            // // Data group control mutexes
            // std::mutex mtx_groupRead {};
            // std::mutex mtx_groupWrite {};
            // std::mutex mtx_borrowed {};


            // Control the updatedness of data
            // Indicates whether or not values have been set before
            int freshness {6};
            // Indicates whether new values have been set. Set to true when putting in new values (PoseEstimator), false when reading (PathFollower)
            bool updated {false};

            // Current robot action. Used for sensor data filtering
            bool isTurning {false};
            bool isDriving {false};
            std::mutex mtx_actionControl {};

            // Front obstacle control
            double frontObstacleDist {-1};
            std::mutex mtx_frontObstacle {};

            // Flushing control
            bool shouldflush {false};
            std::mutex mtx_flush {};

            
        public:
            // Default constructor
            PoseCommunicator();
            // Copy constructor
            // PoseCommunicator(const PoseCommunicator& pComm);

            // Copy the whole structure of frames.
            // PoseCommunicator& operator=(const PoseCommunicator& pComm);
            
            PoseDataSyncBlob borrowData();
            void giveBackData(PoseDataSyncBlob);



            // Getters, setters and calculations

            // Take in a new speed and calculate robotSpeedAvg based on this.
            void calcRobotSpeedAvg(CoordinateFrame newSpeed);


            // // Control the target frame (used by PathFollower)
            // void setTargetFrameTS(CoordinateFrame& frame);
            // CoordinateFrame getTargetFrame();
            // void setTargetFrameTransformTS(Transform tf);
            
            // Returns true if the robot is not on the same tile as startLocalTileFrame
            bool hasDrivenStep();


            // Control what action the robot is currently doing.
            void setTurning(bool turning);
            bool getTurning();
            void setDriving(bool driving);
            bool getDriving();

            // Communication about the front obstacle distance
            void setFrontObstacleDist(double dist);
            double getFrontObstacleDist();


            // Call to flush the pose data
            void flushPose();
            // Call to get if you want to flush the data
            bool getShouldFlushPose();
            // Call to signal that the flush has been performed
            void flushDone();


    };
}
