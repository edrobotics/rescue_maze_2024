#include "transformations/CoordinateFrame.h"


CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame, Transform tf)
{
    setParentTS(parentFrame);
    mtx_general.lock();
    transform = tf;
    mtx_general.unlock();
}

CoordinateFrame::CoordinateFrame(CoordinateFrame* parentFrame)
{
    CoordinateFrame{parentFrame, Transform{}};
}


CoordinateFrame::~CoordinateFrame()
{
    mtx_general.lock();
    // Delete all children of this CF
    deleteChildren();

    // Unregister this CF as a child of the parent
    unregisterMeAsChild();
    mtx_general.unlock();
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
    mtx_general.lock();
    transform = otherFrame.transform;
    parent = otherFrame.parent;
    children = otherFrame.children;
    mtx_general.unlock();
    return *this;
}


CoordinateFrame CoordinateFrame::getWithoutChildren()
{
    // Do a copy
    CoordinateFrame childless {*this};
    mtx_general.lock();

    // Remove the children
    childless.stripChildren();

    mtx_general.unlock();

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
    mtx_general.lock();
    std::cout << "Child num is: " << children.size() << '\n';
    mtx_general.unlock();
}


CoordinateFrame* CoordinateFrame::getParent()
{
    return parent;
}

void CoordinateFrame::setParentTS(CoordinateFrame* newParent)
{
    mtx_general.lock();
    // Unregister child from parent
    unregisterMeAsChild();

    // Change the parent
    parent = newParent;

    // Re-register the child with the new parent
    registerMeAsChild();
    mtx_general.unlock();
}

void CoordinateFrame::registerChild(CoordinateFrame* child)
{
    mtx_general.lock();
    // Add the child
    children.push_back(child);

    mtx_general.unlock();
}

void CoordinateFrame::unregisterChild(CoordinateFrame* child)
{
    mtx_general.lock();
    // Find the child and erase it
    children.erase(std::remove(children.begin(), children.end(), child), children.end());

    mtx_general.unlock();
}

void CoordinateFrame::registerMeAsChild()
{
    mtx_general.lock();
    if (parent!=nullptr)
    {
        parent->registerChild(this);
    }
    mtx_general.unlock();

}

void CoordinateFrame::unregisterMeAsChild()
{
    mtx_general.lock();
    if (parent!=nullptr)
    {
        parent->unregisterChild(this);
    }
    mtx_general.unlock();

}


Transform CoordinateFrame::getRootTransformTS()
{
    mtx_general.lock();
    if (parent==nullptr) // We are at the root and cannot traverse further up the tree
    {
        mtx_general.unlock();
        return transform;
    }
    Transform resultTf {parent->getRootTransformTS() + transform};
    mtx_general.unlock();
    return resultTf;
}


Transform CoordinateFrame::getLevelTransform(int level)
{
    Transform result {};
    // The last level to return (or reached end of tree)
    mtx_general.lock();
    if (level == 1 || parent==nullptr)
    {
        result = transform;
    }
    // Invalid input - fallback to root transform
    else if (level<=0)
    {
        mtx_general.unlock(); // To make getRootTransformTS to work?
        result = getRootTransformTS();
        mtx_general.lock();
    }
    // Normal behaviour.
    else
    {
        result = parent->getLevelTransform(level-1);
    }
    mtx_general.unlock();
    return result;
}

Transform CoordinateFrame::getTransformRootTo(CoordinateFrame* destFrame)
{
    Transform t1 {getRootTransformTS()};
    Transform t2 {destFrame->getRootTransformTS()};
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
    mtx_general.lock();
    // We are there
    if (parent==destFrame)
    {
        return transform;
    }

    // We are at the root transform and cannot continue any longer. Should never happen! (Legitimate cases caught above)
    assert(parent!=nullptr);
    
    Transform resultTf {parent->getTransformUpTo(destFrame)};

    mtx_general.unlock();

    return resultTf;
    
}

// void CoordinateFrame::transformTo(CoordinateFrame* destFrame)
// {
//     transform = getTransformRootTo(destFrame);
//     parent = destFrame;
// }

void CoordinateFrame::transformRootTo(CoordinateFrame* destFrame)
{
    transform = getTransformRootTo(destFrame);
    setParentTS(destFrame);
}

void CoordinateFrame::transformLevelTo(CoordinateFrame* destFrame, int level1, int level2)
{
    transform = getTransformLevelTo(destFrame, level1, level2);
    setParentTS(destFrame);
}

void CoordinateFrame::transformUpTo(CoordinateFrame* destFrame)
{
    transform = getTransformUpTo(destFrame);
    setParentTS(destFrame);
}


void CoordinateFrame::ghostMove(Transform tf)
{
    mtx_general.lock();
    incrementTransfrom(tf);
    for (auto& child: children)
    {
        child->applyTransform(tf.inverse()+child->transform);
    }
    mtx_general.unlock();
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
    #warning why level 1 here? Should be noted. Only works if they have the same parent.
    return calcDist2dLevel(f1, 1, f2, 1);
}

double CoordinateFrame::calcDist2dLevel(CoordinateFrame* f1, int level1, CoordinateFrame* f2, int level2)
{
    Transform res {f2->getTransformLevelTo(f1, level1, level2)};
    return sqrt(pow(res.pos_x, 2) + pow(res.pos_y, 2));
}

std::ostream& operator << (std::ostream& os, CoordinateFrame& frame)
{
    frame.mtx_general.lock();
    os << frame.transform;
    frame.mtx_general.unlock();
    // os << "p_x" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_x << " "
    // << "p_y" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_y << " "
    // << "p_z" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.pos_z << " "

    // << "r_x" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_x << " "
    // << "r_y" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_y << " "
    // << "r_z" << std::setfill('0') << std::setw(3) << std::fixed << std::setprecision(1) << frame.transform.rot_z << " ";

    return os;
}