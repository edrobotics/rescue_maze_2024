#include "localNav/Interpolator.h"


std::vector<int> Interpolator::linear(int startVal, int endVal, int resolution)
{
    std::vector<int> retVal {};
    // Set the increment
    int increment = resolution * ((startVal<=endVal) ? 1 : -1);

    // Computation
    bool done {false};
    for (int i=startVal; !done; i+=increment)
    {
        retVal.push_back(i);
        if (startVal<=endVal)
        {
            if (i>=endVal)
            {
                done = true;
            }
        }
        else
        {
            if (i<=endVal)
            {
                done = true;
            }
        }
    }


    return retVal;


}