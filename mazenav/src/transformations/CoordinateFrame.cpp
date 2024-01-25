#include "transformations/CoordinateFrame.h"


CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame, Transform tf)
: parent {parentFrame}
{
    transform = tf;
}

CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame)
{
    CoordinateFrame{parentFrame, Transform{}};
}

Transform CoordinateFrame::getRootTransform()
{
    if (parent==nullptr) // We are at the root and cannot traverse further up the tree
    {
        return transform;
    }
    else
    {
        return parent->getRootTransform() + transform;
    }
}

Transform CoordinateFrame::transformTo(CoordinateFrame& destFrame)
{
    Transform t1 {getRootTransform()};
    Transform t2 {destFrame.getRootTransform()};
    return t1-t2;
}

void CoordinateFrame::applyTransform(Transform& tf)
{
    transform = tf;
}

void CoordinateFrame::incrementTransfrom(Transform& tf)
{
    transform+=tf;
}

std::ostream& operator << (std::ostream& os, CoordinateFrame& frame)
{
    os << frame.transform;
    // os << "p_x" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_x << " "
    // << "p_y" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_y << " "
    // << "p_z" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_z << " "

    // << "r_x" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_x << " "
    // << "r_y" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_y << " "
    // << "r_z" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_z << " ";

    return os;
}