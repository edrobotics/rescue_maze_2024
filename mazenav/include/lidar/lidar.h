#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "lidar/lidarDataStructures.h"
#include "lidar/lidarDataGetter.h"
#include "lidar/lidarLineMaker.h"
#include "lidar/lidarLineAnalyser.h"

class Lidar
{
private:
    LidarDataGetter lidarDataGetter;
public:
    
    LidarTilePose getLidarPose();
};