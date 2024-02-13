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


Transform CoordinateFrame::getLevelTransform(int level)
{
    // The last level to return
    if (level == 1)
    {
        return transform;
    }
    
    // You have gone too far
    if (level<=0)
    {
        return getRootTransform();
    }
    
    // Normal behaviour.
    return parent->getLevelTransform(level-1);
}

Transform CoordinateFrame::getTransformRootTo(CoordinateFrame* destFrame)
{
    Transform t1 {getRootTransform()};
    Transform t2 {destFrame->getRootTransform()};
    return t1-t2;
}

Transform CoordinateFrame::getTransformLevelTo(CoordinateFrame* destFrame, int level1, int level2)
{

    Transform t1 {};
    Transform t2 {};

    // Invalid levels or intentional. Just get the root for compatibility reasons.
    if (level1<0 || level2<0)
    {
        return getTransformRootTo(destFrame);
    }
    else
    {
        t1 = getLevelTransform(level1);
        t2 = destFrame->getLevelTransform(level2);
        return t1-t2;
    }

}

Transform CoordinateFrame::getTransformUpTo(CoordinateFrame* destFrame)
{
    
    // We are there
    if (parent==destFrame)
    {
        return transform;
    }

    // We are at the root transform and cannot continue any longer. Should never happen! (Legitimate cases caught above)
    assert(parent!=nullptr);
    
    
    return parent->getTransformUpTo(destFrame);
    
}

#warning update with transform variants
void CoordinateFrame::transformTo(CoordinateFrame* destFrame)
{
    transform = getTransformRootTo(destFrame);
    parent = destFrame;
}

void CoordinateFrame::applyTransform(Transform tf)
{
    transform = tf;
}

void CoordinateFrame::incrementTransfrom(Transform tf)
{
    transform+=tf;
}

double CoordinateFrame::calcDist2d(CoordinateFrame f1, CoordinateFrame f2)
{
    Transform res {f2.transform-f1.transform};
    return sqrt(res.pos_x*res.pos_x + res.pos_y*res.pos_y);
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