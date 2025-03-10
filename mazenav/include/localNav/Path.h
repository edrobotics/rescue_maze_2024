#pragma once
#include <vector>

#include "transformations/tfsys.h"
#include "localNav/PathFrame.h"
// #include "localNav/Interpolator.h"

class Path
{
    public:

        // void interpolate();

        CoordinateFrame parentFrame {nullptr};

        // The key frames that the robot should hit during the path. The frames are relative the first path point
        std::vector<PathFrame> keyFrames {};
        void addKeyFrame(PathFrame frame);

        // The poses that make the path when interpolated. The actual thing to follow
        // std::vector<CoordinateFrame> interpolatedFrames {};

        // The resolution for the interpolation (maybe as argument instead?)

    private:
        // Default resolution for interpolation. [unit]/step
        // int interpolRes {1};
        // Interpolate between points with the given resolution.
        // void interpolate(int res);

        // Interpolator interpolator {};

};