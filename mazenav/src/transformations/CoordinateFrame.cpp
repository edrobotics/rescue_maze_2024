#include "transformations/CoordinateFrame.h"


CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame, Transform tf)
{
    setParent(parentFrame);
    transform = tf;
}

CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame)
{
    CoordinateFrame{parentFrame, Transform{}};
}


CoordinateFrame::~CoordinateFrame()
{
    // Delete all children of this CF
    deleteChildren();

    // Unregister this CF as a child of the parent
    unregisterMeAsChild();
}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& frame)
{
    // transform = frame.transform;
    // parent = frame.parent;
    // children = frame.children;
    *this = frame;
}

CoordinateFrame CoordinateFrame::operator=(const CoordinateFrame& otherFrame)
{
    transform = otherFrame.transform;
    parent = otherFrame.parent;
    children = otherFrame.children;

    return *this;
}


CoordinateFrame CoordinateFrame::getWithoutChildren()
{
    // Do a copy
    CoordinateFrame childless {*this};

    // Remove the children
    childless.stripChildren();

    // Return the child-less copy
    return childless;
}


void CoordinateFrame::deleteChildren()
{
    
    while (!children.empty())
    {
        CoordinateFrame* tmp = children.front();
        children.erase(children.begin());
        delete tmp;
    }

    // children.clear();
}

void CoordinateFrame::stripChildren()
{
    children.clear();
}

void CoordinateFrame::printChildNum()
{
    std::cout << "Child num is: " << children.size() << '\n';
}


CoordinateFrame* CoordinateFrame::getParent()
{
    return parent;
}

void CoordinateFrame::setParent(CoordinateFrame* newParent)
{
    // Unregister child from parent
    unregisterMeAsChild();

    // Change the parent
    parent = newParent;

    // Re-register the child with the new parent
    registerMeAsChild();
}

void CoordinateFrame::registerChild(CoordinateFrame* child)
{
    // Add the child
    children.push_back(child);
}

void CoordinateFrame::unregisterChild(CoordinateFrame* child)
{
    // Find the child and erase it
    children.erase(std::remove(children.begin(), children.end(), child), children.end());
}

void CoordinateFrame::registerMeAsChild()
{
    if (parent!=nullptr)
    {
        parent->registerChild(this);
    }

}

void CoordinateFrame::unregisterMeAsChild()
{
    if (parent!=nullptr)
    {
        parent->unregisterChild(this);
    }

}


Transform CoordinateFrame::getRootTransform()
{
    if (parent==nullptr) // We are at the root and cannot traverse further up the tree
    {
        return transform;
    }
    
    return parent->getRootTransform() + transform;
}


Transform CoordinateFrame::getLevelTransform(int level)
{
    // The last level to return (or reached end of tree)
    if (level == 1 || parent==nullptr)
    {
        return transform;
    }
    
    // Invalid input - fallback to root transform
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

// void CoordinateFrame::transformTo(CoordinateFrame* destFrame)
// {
//     transform = getTransformRootTo(destFrame);
//     parent = destFrame;
// }

void CoordinateFrame::transformRootTo(CoordinateFrame* destFrame)
{
    transform = getTransformRootTo(destFrame);
    setParent(destFrame);
}

void CoordinateFrame::transformLevelTo(CoordinateFrame* destFrame, int level1, int level2)
{
    transform = getTransformLevelTo(destFrame, level1, level2);
    setParent(destFrame);
}

void CoordinateFrame::transformUpTo(CoordinateFrame* destFrame)
{
    transform = getTransformUpTo(destFrame);
    setParent(destFrame);
}


void CoordinateFrame::ghostMove(Transform tf)
{
    incrementTransfrom(tf);
    for (auto& child: children)
    {
        child->applyTransform(tf.inverse()+child->transform);
    }
}


void CoordinateFrame::applyTransform(Transform tf)
{
    transform = tf;
}

void CoordinateFrame::incrementTransfrom(Transform tf)
{
    transform+=tf;
}

double CoordinateFrame::calcDist2d(CoordinateFrame* f1, CoordinateFrame* f2)
{
    return calcDist2dLevel(f1, 1, f2, 1);
}

double CoordinateFrame::calcDist2dLevel(CoordinateFrame* f1, int level1, CoordinateFrame* f2, int level2)
{
    Transform res {f2->getTransformLevelTo(f1, level1, level2)};
    return sqrt(pow(res.pos_x, 2) + pow(res.pos_y, 2));
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