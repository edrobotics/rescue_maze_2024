#include "localNav/Interpolator.h"


std::vector<int> Interpolator::linear(int startVal, int endVal, int steps)
{
    std::vector<int> retVal {};
    // Set the increment
    double increment = (endVal-startVal)/static_cast<double>(steps);

    // Computation
    for (int i=1; i<(steps+1); ++i)
    {
        retVal.push_back(static_cast<int>(startVal + i*increment));
    }

    return retVal;


}