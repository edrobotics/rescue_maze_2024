#pragma once

#include "transformations/tfsys.h"

class PathFrame
{
    public:

        PathFrame(CoordinateFrame cf)
        : frame{cf}
        {

        }
        // The CoordinateFrame
        CoordinateFrame frame;

        // The radius (in mm) through which the robot must pass for the pathpoint to be considered visited
        int visitedRadius {20};

};