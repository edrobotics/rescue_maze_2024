#pragma once

#include <array>

class SingleTofData
{
    public:

        // Sets the current value and calculates the average.
        void setCur(int newVal);

        int cur {0};
        double avg {0};

    private:
        constexpr static int HISTORY_NUM {3};
        std::array<int, HISTORY_NUM> history {};

        // Updates the average with newVal
        void updateAvg(int newVal);

        // The current index of the value in updateAvg
        int historyIndex {0};
        // Increment the historyIndex with wrapping
        void incrementHistoryIndex();


};