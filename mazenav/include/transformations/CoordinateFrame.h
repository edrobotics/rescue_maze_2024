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
    Transform transformTo(CoordinateFrame& destFrame);

    // Set the transform
    void applyTransform(Transform& tf);

    // Increment the transform by a transform
    void incrementTransfrom(Transform& tf);

    private:
    Transform getRootTransform();

};

std::ostream& operator << (std::ostream& os, CoordinateFrame& frame);