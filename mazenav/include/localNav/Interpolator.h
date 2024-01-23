#pragma once

#include <vector>

class Interpolator
{
    public:
        // Interpolates linearly between two values
        std::vector<int> linear(int starVal, int endVal, int resolution);

        // Interpolates <???> between two values
        std::vector<int> cubic(int startVal, int endVal, int resolution);

    private:

};