#include "localNav/PathPlanner.h"

// void PathPlanner::setPath(Path path)
// {
//     this->path = path;
// }

void PathPlanner::setStartFrame(CoordinateFrame frame)
{
    startFrame = frame.getWithoutChildren();
}

void PathPlanner::setGlobalPath(std::vector<communication::DriveCommand> globalPath)
{
    this->globalPath = globalPath;
}


void PathPlanner::calculate()
{
    // Add initial keypose
    // path.addKeyFrame(startFrame);
    
    // Add the remaining keyposes.
    fillKeyFrames(globalPath);

    // Interpolate between these coordinates
    path.interpolate();
}

void PathPlanner::fillKeyFrames(std::vector<communication::DriveCommand>& globPath)
{
    CoordinateFrame* pathParent {&(startFrame)};
    path.parentFrame = pathParent->getWithoutChildren();
    CoordinateFrame curFrame {pathParent};
    path.addKeyFrame(curFrame);
    for (communication::DriveCommand move : globPath)
    {
        curFrame.setParent(&(path.keyFrames.back()));

        switch (move)
        {
            case communication::driveForward:
                curFrame.incrementTransfrom(tfForward);
                break;
            case communication::turnLeft:
                curFrame.incrementTransfrom(tfLeft);
                break;
            case communication::turnRight:
                curFrame.incrementTransfrom(tfRight);
                break;
            default:
                break;
        }
        // Change the relative commands in globalPath to absolute coordinates with help of the transform system
        curFrame.transformUpTo(pathParent);
        path.addKeyFrame(curFrame);
    }

}

Path PathPlanner::getPath()
{
    return path;
}