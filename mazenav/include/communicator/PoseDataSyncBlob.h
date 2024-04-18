#pragma once

#include <array>
#include <mutex>

#include "transformations/tfsys.h"
#include "GlobalConstants.h"


namespace communication
{
    class PoseDataSyncBlob
    {
        private:
            // For timed average of robotSpeed
            static constexpr int HISTORY_NUM {4};
            int historyIndex {0};
            void incrementHistoryIndex();
            void calcAverage(CoordinateFrame& result);
            std::array<CoordinateFrame, HISTORY_NUM> robotSpeeds {create_array<HISTORY_NUM, CoordinateFrame>(CoordinateFrame{nullptr})};


            static constexpr int INITIAL_FRESHNESS {6};


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

            // Mutexes
            // If locked, cannot write to the object (set when borrowing)
            std::mutex mtx_writeRights {};
            // If locked, cannot read object (set when writing)
            std::mutex mtx_readRights {};

        
        public:

            PoseDataSyncBlob();

            PoseDataSyncBlob(const PoseDataSyncBlob& pdBlob);
            PoseDataSyncBlob& operator= (const PoseDataSyncBlob& pdBlob);


            PoseDataSyncBlob borrow();
            PoseDataSyncBlob getCopy();
            void giveBack(PoseDataSyncBlob blob);
            void giveBackDummyData();

            void setTargetTransformTS(Transform tf);

            // void resetFreshness();
            int getFreshness();
            // void decrementFreshness();


            // Getters
            CoordinateFrame getWorldFrame();
            CoordinateFrame getLocalTileFrame();
            CoordinateFrame getRobotFrame();
            CoordinateFrame getLastRobotFrame();
            std::chrono::steady_clock::time_point getCurRobotTime();
            std::chrono::steady_clock::time_point getLastRobotTime();
            CoordinateFrame getRobotSpeedAvg();
            CoordinateFrame getRobotSpeedCur();
            CoordinateFrame getTargetFrame();
            CoordinateFrame getStartLocalTileFrame();


            void calcRobotSpeedAvg(CoordinateFrame newSpeed);


            // The variables themselves

            // Position frames
            CoordinateFrame worldFrame {nullptr};
            CoordinateFrame localTileFrame {&worldFrame};
            CoordinateFrame robotFrame {&localTileFrame};
            CoordinateFrame lastRobotFrame {&localTileFrame};
            // Time point for when the lastRobotTime was captured
            std::chrono::steady_clock::time_point curRobotTime {std::chrono::steady_clock::now()};
            std::chrono::steady_clock::time_point lastRobotTime {std::chrono::steady_clock::now()};

            // Speed frames (could do with parents instead, but this works for now)
            CoordinateFrame robotSpeedAvg {nullptr};
            CoordinateFrame robotSpeedCur {nullptr};

            // Used by PathFollower. Is here to ensure synchronisation
            CoordinateFrame targetFrame {&localTileFrame};

            // The localTileFrame which was the current one when starting driving.
            CoordinateFrame startLocalTileFrame {&worldFrame};

            // Indicates whether or not values have been set before. The vlaue is how many left until considered initialized.
            int freshness {INITIAL_FRESHNESS};
    };
}