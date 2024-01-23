#pragma once
#include <vector>

#include "communicator/RobotPose.h"
#include "localNav/Interpolator.h"

class Path
{
    public:

        void interpolate();

        // The key poses that the robot should hit during the path
        #warning Replace with PathPoints, containing RobotPose, enforcement rules (for example angle) and possibly speeds
        std::vector<RobotPose> keyPoses {};
        void addKeyPose(RobotPose pose);

        // The poses that make the path when interpolated. The actual thing to follow
        std::vector<RobotPose> interpolatedPoses {};

        // The resolution for the interpolation (maybe as argument instead?)

    private:
        // Default resolution for interpolation
        int interpolRes {1};
        // Interpolate between points with the given resolution.
        void interpolate(int res);

        Interpolator interpolator {};

};