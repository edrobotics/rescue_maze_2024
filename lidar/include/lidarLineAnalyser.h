#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include <array> // DELET THIS

#include <opencv2/opencv.hpp>

#include <lidarCoordinate.h>

#define TILE_READ_AMOUNT 9 //MUST BE ODD

class LineAnalyser
{
    public:
    LineAnalyser(std::vector<cv::Vec<cv::Point, 2>> lines);

    double getOrientation();
    cv::Point getTilePosition();
    std::array<std::array<cv::Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> getMap(); //We need better way. Map object?
    // int getInts();

    private:
    void calcOrientation();
    void calcPosition(); //AFTER ORIENTATION
    void calcMap(); //AFTER POSITION
    //Straight line class, just a straight line with extra information calculated
    class SLine
    {
        public:
        SLine(cv::Vec<cv::Point, 2> linePoints);
        double orientation; //The orientation of the line, in radians, compared to the X-axis and between M_PI/2 and -M_PI/2
        double length; //The length of the line

        cv::Point startPoint; //The starting point of the line
        cv::Point endPoint; //The endpoint of the line

        cv::Point closestToOrigin; //The point that would be closest to the origin if the line was infinitly long
    };

    double orientation;
    cv::Point position;
    std::array<std::array<cv::Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> relMap  = {{{false, false, false, false}}};

    std::vector<SLine> anaLines; //All lines
    std::vector<SLine> wallLines; //Likely walls

    bool orientationCalced = false;
    bool positionCalced = false;
    bool mapCalced = false;
};