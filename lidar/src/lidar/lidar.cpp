#include "lidar/lidar.h"

LidarData Lidar::getLidarPose()
{
    LidarLineMaker lineMaker(lidarDataGetter.getData());
    LidarLineAnalyser lineAnalyser(lineMaker.getLines());

    cv::Point position = lineAnalyser.getTilePosition();
    double orientation = lineAnalyser.getOrientation();
    auto localMazeMap = lineAnalyser.getMap();
    
    std::cout << "Pos: " << position.x << " , " << position.y << " :degree: " << orientation*180/M_PI;
    std::cout << "This WASD: " << localMazeMap.relativeMap[4][4][0] << localMazeMap.relativeMap[4][4][1] 
                               << localMazeMap.relativeMap[4][4][2] << localMazeMap.relativeMap[4][4][3] << std::endl;
    LidarData lidarData;
    lidarData.relativeTilePose.relativeX = position.x;
    lidarData.relativeTilePose.relativeY = position.y;
    lidarData.relativeTilePose.orientationInRadians = orientation;
    lidarData.localTiles = localMazeMap;

    return lidarData;
}