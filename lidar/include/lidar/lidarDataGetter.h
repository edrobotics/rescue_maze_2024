#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <math.h>

#include <ldlidar_driver/ldlidar_driver_linux.h>

#include <lidar/lidarFiles.h>

// #define CODE_READ_LIDAR

#ifndef CODE_READ_LIDAR
#define CODE_READ_FILE_TXT
// #define CODE_READ_FILE_IMG
#else
#define CODE_SAVE_TXT
#define CODE_SAVE_SCAN
// #define CODE_SAVE_LATEST
#endif

#define PORT "/dev/ttyUSB0" //COM4?
#define BAUDRATE 230400U

class LidarDataGetter
{
    public:
    LidarDataGetter();
    ~LidarDataGetter();

    ldlidar::Points2D getData();

    private:
    void createCoords(ldlidar::Points2D& points);

    ldlidar::LDLidarDriverLinuxInterface ldInterface;
    std::chrono::_V2::system_clock::time_point lastLidarUseTime;

    int64_t lidarScanTimeMS = 1000/10;
};