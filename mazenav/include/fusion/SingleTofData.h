#pragma once

#include <array>
#include <iostream>

class SingleTofData
{
    public:

        // Sets the current value and calculates the average.
        void setCur(int newVal);

        int cur {0};
        double avg {0};

    private:
        // How many values to keep in history
        constexpr static int HISTORY_NUM {4};
        // How many of the most recent values not to count
        constexpr static int HISTORY_EXCLUSION_NUM {1};
        std::array<int, HISTORY_NUM> history {};

        // Updates the average with newVal
        void updateAvg(int newVal);

        // The current index of the value in updateAvg
        int historyIndex {0};
        // Increment the historyIndex with wrapping
        void incrementHistoryIndex();
        // Wraps the index to allowed range (0 - HISTORY_NUM)
        int wrapIndex(int index);


};