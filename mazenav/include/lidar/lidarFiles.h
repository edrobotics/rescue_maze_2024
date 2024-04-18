#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <lidar/ldlidar_driver/ldlidar_datatype.h>

// #ifdef PI
#define PROJECTPATH std::string("/home/theseus/RCJ2024/lidar/")
// #else
// #define PROJECTPATH std::string("/home/markfri/code/rescue_maze_2024/lidar/")
// #endif
// #define SCANPATH PROJECTPATH + std::string("img/scans/")
#define BASEIMGPATH PROJECTPATH + std::string("img/base.png")
// #define SCAN_BASENAME SCANPATH + std::string("ldlidar_scan_")
// #define SCAN_EXAMPLE_CURRENT PROJECTPATH + std::string("img/example/ldlidar_scan_09")
// #define SCANTXT_PATH SCAN_EXAMPLE_CURRENT + std::string(".txt") //PROJECTPATH + std::string("img/example/ldlidar_scan_02.txt") //SCANPATH + std::string("ldlidar_scan_latest.txt")

#ifdef CODE_READ_FILE_IMG
#define READ_FILE_IMG_PATH SCAN_EXAMPLE_CURRENT + std::string(".png") //SCAN_BASENAME + std::string("latest.png")
#endif

// void writeFile(ldlidar::Points2D& points);

ldlidar::Points2D pointsFromTxt(std::string path);