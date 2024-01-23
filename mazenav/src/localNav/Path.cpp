#include "localNav/Path.h"


void Path::interpolate()
{
    // Iterate over the key poses
    for (auto iter = keyPoses.begin(); iter != keyPoses.end();) // Explicitly does not increment iterator in loop head. Happens in the body.
    {
        // Interpolate all variables between the current and next iteration

        // Interpolation
        RobotPose startPose = *iter;
        RobotPose endPose = *(++iter); // Increment the operator and also store the element. IMPORTANT that ++ is before iter (otherwise new value would not be stored)

        std::vector<int> t_x = interpolator.linear(startPose.t_x, endPose.t_x, interpolRes);
        std::vector<int> t_y = interpolator.linear(startPose.t_y, endPose.t_y, interpolRes);
        std::vector<int> r_z = interpolator.linear(startPose.r_z, endPose.r_z, interpolRes);

        // Problem: I want them all to scale together. Either need to stretch or interpolate them together.

    }
};