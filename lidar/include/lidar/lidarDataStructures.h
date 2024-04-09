#pragma once

#include <array>
#include <opencv2/opencv.hpp>

#define TILE_READ_AMOUNT 9 //MUST BE ODD

struct LocalTileMap
{
    std::array<std::array<cv::Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> relativeMap = {{{false, false, false, false}}};
};

struct LidarTilePose
{
    double orientationInRadians;
    int relativeX;
    int relativeY;
};


struct LidarData
{
    LocalTileMap localTiles;
    LidarTilePose relativeTilePose;
};