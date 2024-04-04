#pragma once

#include "transformations/Transform.h"
#include <iomanip>
#include <cassert>
#include <vector>
#include <algorithm>
#include <mutex>

class CoordinateFrame
{
    public:
    CoordinateFrame(CoordinateFrame* parentFrame, Transform tf);
    CoordinateFrame(CoordinateFrame* parentFrame);

    ~CoordinateFrame();

    // Copy constructor
    CoordinateFrame(const CoordinateFrame& frame);
    // Assignment operator
    CoordinateFrame operator=(const CoordinateFrame& otherFrame);

    // Enables child-less copying. Returns the current object but without children.
    CoordinateFrame getWithoutChildren();
    // Strip the children from this object, but keep them in memory for everyone else
    void stripChildren();


    Transform transform {};

    // gets the parent of the object
    CoordinateFrame* getParent();
    // TS. Sets the parent of the object.
    void setParentTS(CoordinateFrame* newParent);

    // Delete the children
    void deleteChildren();
    // Print the number of children of the object
    void printChildNum();

    // std::vector<CoordinateFrame*> getChildren();

    // Get the transform between this frame and the destination frame = get the position of this frame in relation to destFrame
    // Transversal up to the root frame
    Transform getTransformRootTo(CoordinateFrame* destFrame);
    // Transversal for level1 and level 2 steps - you need know the frame tree and specify the levels. With levels specified wrong, transform will be incorrect.
    // level1 - how many steps to travel up for the object being acted on
    // level2 - same as level1 but for destFrame
    Transform getTransformLevelTo(CoordinateFrame* destFrame, int level1, int level2);
    // Transform straight up the tree until you get to destFrame
    Transform getTransformUpTo(CoordinateFrame* destFrame);

    // Set a new parent and change the transform to be relative that new parent
    // void transformTo(CoordinateFrame* destFrame);
    void transformRootTo(CoordinateFrame* destFrame);
    void transformLevelTo(CoordinateFrame* destFrame, int level1, int level2);
    void transformUpTo(CoordinateFrame* destFrame);

    // Move this CF but keep parent and children's global coordinates the same.
    // tf - the transform describing relative movement of the frame
    void ghostMove(Transform tf);

    // Set the transform
    void applyTransform(Transform tf);

    // Increment the transform by a transform
    void incrementTransfrom(Transform tf);

    // Calculate the euclidean distances between two coordinateframes
    static double calcDist2d(CoordinateFrame* f1, CoordinateFrame* f2);
    static double calcDist2dLevel(CoordinateFrame* f1, int level1, CoordinateFrame* f2, int level2);

    
    // Mutex to handle concurrency
    std::mutex mtx_general {};

    private:
    // The parent CF
    CoordinateFrame* parent {};
    // The children CF
    std::vector<CoordinateFrame*> children;


    // Register a child in the current object
    void registerChild(CoordinateFrame* child);
    // Register the current object as a child in the parent
    void registerMeAsChild();

    // Unregister a child in the current object
    void unregisterChild(CoordinateFrame* child);
    // Unregister this object from the parent
    void unregisterMeAsChild();

    // Get the transform up to root CF.
    Transform getRootTransformTS();
    // Get the transform <level> steps up. A level of 1 means just getting the transform of the CF.
    Transform getLevelTransform(int level);

};

std::ostream& operator << (std::ostream& os, CoordinateFrame& frame);

