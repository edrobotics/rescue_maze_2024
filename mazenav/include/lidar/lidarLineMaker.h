#pragma once

#include <vector>
#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "lidar/ldlidar_driver/ldlidar_datatype.h"

#include "lidar/lidarCoordinate.h"
#include "lidar/lidarFiles.h"

class LidarLineMaker
{
    public:
    LidarLineMaker(ldlidar::Points2D points);
    LidarLineMaker(ldlidar::Points2D points, cv::Mat& debugOut);

    /** @brief Gets lines that were formed by the points
     * @retval The lines
    **/
    std::vector<cv::Vec<cv::Point, 2>> getLines();

    private:
    std::vector<cv::Vec<cv::Point, 2>> combinedLines;
    
    cv::Mat drawLines(ldlidar::Points2D& points, std::vector<cv::Vec4i>& linesOut);
    void mergeLines(std::vector<cv::Vec4i>& lines, std::vector<cv::Vec<cv::Point, 2>>& out);

    /** @brief Merges lines if they are within thresholds
    * @param line_i First line for merging
    * @param line_j Second line for merging
    * @param output Output vector
    * @param angleThresh The maximum angle difference between the lines that should be merged
    * @param distThresh The maximum gap between lines that should be merged
    * @param extraLogging Whether extra logging should be used. Should be false normally.
    * @retval True if merged, false if not.
    **/
    bool mergeLineSegments(const cv::Vec<cv::Point, 2>& line_i, const cv::Vec<cv::Point, 2>& line_j, cv::Vec<cv::Point, 2>& output, double angleThresh = M_PI/6, double distThresh = 100, bool extraLogging = false);
};