#include <opencv2/opencv.hpp>

#include <lidar/ldlidar_driver/ldlidar_driver_linux.h>

#include <iostream>
#include <chrono>
#include <math.h>

#include <lidar/lidar.h>

using namespace std;
using namespace ldlidar;
using namespace cv;

void displayImage(Mat& lineImg, LidarLineAnalyser& lAnalyser);


int main() //Expandera mer i riktningen åt senare och tidigare punkter? (om de är sorterade efter vinkel i point2d)
{           //Read/write intensity in txt
    Lidar lidar;

    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();

        lidar.getLidarData();

        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Full analysis:" << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "µs" << std::endl;
    }

    // displayImage(showLines, lAnalyser);

    //**************************************   SAVING FILES    **************************************//

    string tNow = to_string(time(0));
    #ifdef CODE_SAVE_SCAN
    imwrite(SCAN_BASENAME + tNow + string(".png"), image);
    #endif
    #ifdef CODE_SAVE_SCANPLT
    imwrite(SCAN_BASENAME + tNow + string("_PLT.png"), showLines);
    #endif
    #ifdef CODE_SAVE_LATEST
    imwrite(SCAN_BASENAME + "latest_PLT.png", showLines);
    imwrite(SCAN_BASENAME + "latest.png", image);
    #endif

    #ifdef CODE_SAVE_TXT
    writeFile(points);
    #endif

    return 0;
}

void displayImage(Mat& lineImg, LidarLineAnalyser& lAnalyser)
{
    double orientation = lAnalyser.getOrientation();
    Point relativePosition = lAnalyser.getTilePosition();
    std::array<std::array<cv::Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> relMap = lAnalyser.getMap().relativeMap;

    Mat relativeImg = imread(BASEIMGPATH);

    line(lineImg, Point(0, ORIGIN_Y), Point(IMG_SIZE[0], ORIGIN_Y), Scalar(200,200,200));
    line(lineImg, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE[1]), Scalar(200,200,200));
    line(relativeImg, Point(0, ORIGIN_Y), Point(IMG_SIZE[0], ORIGIN_Y), Scalar(200,200,200));
    line(relativeImg, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE[1]), Scalar(200,200,200));

    for (int i = 0; i < 2000; i += 300)
    {
        line(lineImg, transformPoint(Point(0, ORIGIN_Y + relativePosition.y - i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE[0], ORIGIN_Y + relativePosition.y - i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(lineImg, transformPoint(Point(ORIGIN_X + relativePosition.x - i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + relativePosition.x - i, IMG_SIZE[1]), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(lineImg, transformPoint(Point(0, ORIGIN_Y + relativePosition.y + i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE[0], ORIGIN_Y + relativePosition.y + i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(lineImg, transformPoint(Point(ORIGIN_X + relativePosition.x + i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + relativePosition.x + i, IMG_SIZE[1]), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        
        line(relativeImg, Point(0, ORIGIN_Y - 150 - i), Point(IMG_SIZE[0], ORIGIN_Y - 150 - i), Scalar(200,100,200));
        line(relativeImg, Point(ORIGIN_X - 150 - i, 0), Point(ORIGIN_X - 150 - i, IMG_SIZE[1]), Scalar(200,100,200));
        line(relativeImg, Point(0, ORIGIN_Y + 150 + i), Point(IMG_SIZE[0], ORIGIN_Y + 150 + i), Scalar(200,100,200));
        line(relativeImg, Point(ORIGIN_X + 150 + i, 0), Point(ORIGIN_X + 150 + i, IMG_SIZE[1]), Scalar(200,100,200));
    }

	for (size_t i = 0; i < TILE_READ_AMOUNT; i++)
	{
		for (size_t j = 0; j < TILE_READ_AMOUNT; j++)
		{
			if (relMap[j][i][0])
				line(relativeImg, Point(i*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (relMap[j][i][1])
				line(relativeImg, Point(i*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point(i*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (relMap[j][i][2])
				line(relativeImg, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1-1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (relMap[j][i][3])
				line(relativeImg, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1-1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
		}
	}

    circle(lineImg, Point(ORIGIN_X, ORIGIN_Y), 5, Scalar(255, 120, 255), 3);
    circle(relativeImg, Point(ORIGIN_X, ORIGIN_Y) - relativePosition + Point(150, 150), 5, Scalar(255, 120, 255), 3);

    cv::Range showSize(1000, 4000);

    string winName = "Combined lines (in red), Hough detected lines (in green)";
    namedWindow(winName, WINDOW_NORMAL);
    imshow(winName, lineImg(showSize, showSize));
    resizeWindow(winName, Size(1000, 1000));

    string winName2 = "Relative img";
    namedWindow(winName2, WINDOW_NORMAL);
    imshow(winName2, relativeImg(showSize, showSize));
    resizeWindow(winName2, Size(1000, 1000));
    waitKey(0);
}