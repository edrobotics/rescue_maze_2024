#pragma once

#include "transformations/Transform.h"
#include <iomanip>

class CoordinateFrame
{
    public:
    CoordinateFrame(CoordinateFrame* parentFrame, Transform tf);
    CoordinateFrame(CoordinateFrame* parentFrame);
    CoordinateFrame* parent;
    Transform transform {};

    // Get the transform between this frame and the destination frame = get the position of this frame in relation to destFrame
    Transform getTransformTo(CoordinateFrame* destFrame);

    // Set a new parent and change the transform to be relative that new parent
    void transformTo(CoordinateFrame* destFrame);

    // Set the transform
    void applyTransform(Transform tf);

    // Increment the transform by a transform
    void incrementTransfrom(Transform tf);

    // Calculate the euclidean distances between two coordinateframes
    static double calcDist2d(CoordinateFrame f1, CoordinateFrame f2);

    private:
    Transform getRootTransform();

};

std::ostream& operator << (std::ostream& os, CoordinateFrame& frame);