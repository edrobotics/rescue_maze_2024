#include "localNav/Path.h"


void Path::addKeyFrame(PathFrame frame)
{
    keyFrames.push_back(frame);
}

// void Path::interpolate()
// {

//     // Iterate over the key poses (rather path segments between them)
//     for (auto iter = keyFrames.begin(); iter != keyFrames.end();) // Explicitly does not increment iterator in loop head. Happens in the body.
//     {
//         // Interpolate all variables between the current and next iteration

//         // Interpolation

//         CoordinateFrame startFrame {iter->getWithoutChildren()};
//         ++iter; // Increment the iterator.
//         // Check if end iterator has been reached?
//         if (iter==keyFrames.end())
//         {
//             break;
//         }
//         CoordinateFrame endFrame {iter->getWithoutChildren()};

//         // Determine number of steps
//         double distance = CoordinateFrame::calcDist2d(&startFrame, &endFrame);
//         double rotationDistance = abs(endFrame.transform.rot_z-startFrame.transform.rot_z);
//         int transSteps = distance/interpolRes;
//         int rotSteps = rotationDistance/interpolRes;
//         int steps = (transSteps > rotSteps) ? transSteps : rotSteps;

//         std::vector<int> t_x = interpolator.linear(startFrame.transform.pos_x, endFrame.transform.pos_x, steps);
//         std::vector<int> t_y = interpolator.linear(startFrame.transform.pos_y, endFrame.transform.pos_y, steps);
//         std::vector<int> r_z = interpolator.linear(startFrame.transform.rot_z, endFrame.transform.rot_z, steps); // Steps may be weird. Should handle special case for rotation?

//         // Fill up the interpolated steps vector
//         for (int i=0;i<steps-1;++i) // steps-1 because the last step is the first step in the next keyframe/pathsegment
//         {
//             CoordinateFrame pushbackFrame {&parentFrame};
//             pushbackFrame.transform.pos_x = t_x[i];
//             pushbackFrame.transform.pos_y = t_y[i];
//             pushbackFrame.transform.rot_z = r_z[i];
//             interpolatedFrames.push_back(pushbackFrame);
//         }

//     }

//     interpolatedFrames.push_back(keyFrames.back().getWithoutChildren()); // Set the last keypose as the last interpolated pose
// };