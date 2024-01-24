#include "localNav/Path.h"


void Path::addKeyPose(RobotPose pose)
{
    keyPoses.push_back(pose);
}

void Path::interpolate()
{

    // Iterate over the key poses (rather path segments between them)
    for (auto iter = keyPoses.begin(); iter != keyPoses.end();) // Explicitly does not increment iterator in loop head. Happens in the body.
    {
        // Interpolate all variables between the current and next iteration

        // Interpolation

        RobotPose startPose = *iter;
        RobotPose endPose = *(++iter); // Increment the operator and also store the element. IMPORTANT that ++ is before iter (otherwise new value would not be stored)
        // Check if end iterator has been reached?
        // if (iter==keyPoses.end())
        // {
        //     break;
        // }

        // Determine number of steps
        double distance = RobotPose::calcDistance2d(startPose, endPose);
        double rotationDistance = abs(endPose.r_z-endPose.r_z);
        int transSteps = distance/interpolRes;
        int rotSteps = rotationDistance/interpolRes;
        int steps = (transSteps > rotSteps) ? transSteps : rotSteps;

        std::vector<int> t_x = interpolator.linear(startPose.t_x, endPose.t_x, steps);
        std::vector<int> t_y = interpolator.linear(startPose.t_y, endPose.t_y, steps);
        std::vector<int> r_z = interpolator.linear(startPose.r_z, endPose.r_z, steps); // Steps may be weird. Should handle special case for rotation?

        // Fill up the interpolated steps vector
        for (int i=0;i<steps;++i)
        {
            RobotPose pushbackPose {};
            pushbackPose.t_x = t_x[i];
            pushbackPose.t_y = t_y[i];
            pushbackPose.r_z = r_z[i];
            interpolatedPoses.push_back(pushbackPose);
        }
        interpolatedPoses.pop_back(); // Remove the last keyframe, as it is the start of the next round

    }

    interpolatedPoses.push_back(keyPoses.back()); // Set the last keypose as the last interpolated pose
};