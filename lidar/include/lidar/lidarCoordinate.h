#pragma once

#include <opencv2/opencv.hpp>

const cv::Vec2i IMG_SIZE(5000, 5000);
const int ORIGIN_X = IMG_SIZE[0]/2;
const int ORIGIN_Y = IMG_SIZE[1]/2;
const cv::Point ORIGIN(ORIGIN_X, ORIGIN_Y);

double piMod(double mod);
double pi2Mod(double mod);
int tileMod(int mod, double modCenter);

cv::Point transformPoint(cv::Point p, cv::Point origin, double angle);
