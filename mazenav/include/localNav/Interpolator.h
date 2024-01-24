#pragma once

#include <vector>

class Interpolator
{
    public:
        // Interpolates linearly between two values
        std::vector<int> linear(int starVal, int endVal, int steps);

        // Interpolates <???> between two values
        std::vector<int> cubic(int startVal, int endVal, int steps);

    private:

};