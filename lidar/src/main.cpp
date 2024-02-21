#include <opencv2/opencv.hpp>

#include <ldlidar_driver/ldlidar_driver_linux.h>

#include <iostream>
#include <chrono>
#include <math.h>
// #include <thread>

#include <lidar/lidarDataGetter.h>
#include <lidar/lidarLineAnalyser.h>
#include <lidar/lidarLineMaker.h>

#define CODE_ANALYSIS

#ifdef CODE_ANALYSIS
#define SHOW_IMG

#ifdef SHOW_IMG
#define SHOW_IMG_SIZE cv::Range(1000, 4000)
#define SHOW_CLINES //Merged
#define SHOW_PLINES //Hough output
#endif

// #define CODE_SAVE_SCANPLT
#endif

using namespace std;
using namespace ldlidar;
using namespace cv;


int main() //Expandera mer i riktningen åt senare och tidigare punkter? (om de är sorterade efter vinkel i point2d)
{           //Read/write intensity in txt
            //!!! MIDPOS: SOMETHING IS EXTREMELY SLOW

    // *** STARTUP ***
    //Pre-calculate the wall endpoints on tiles
    // int tileEndPointsXY[TILE_READ_AMOUNT+1];
    // for (size_t i = 0; i < TILE_READ_AMOUNT+1; i++)
    // {
    //     tileEndPointsXY[i] = 300 * i - 300*(TILE_READ_AMOUNT)/2;
    // }
    LidarDataGetter ldGetter;

    //loop
    Points2D points = ldGetter.getData();

    #ifdef CODE_ANALYSIS
    auto start = std::chrono::high_resolution_clock::now();

    Mat showLines;
    LidarLineMaker lMaker(points, showLines);

    vector<Vec<Point, 2>> combinedLines = lMaker.getLines();

    LidarLineAnalyser lAnalyser(combinedLines);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "Hough:" << std::chrono::duration_cast<std::chrono::microseconds>(mergeStart-houghStart).count() << "µs" << '\n';
    // std::cout << "Merge:" << std::chrono::duration_cast<std::chrono::microseconds>(orientStart-mergeStart).count() << "µs" << '\n';
    // std::cout << "Orient:" << std::chrono::duration_cast<std::chrono::microseconds>(coordStart-orientStart).count() << "µs" << '\n';
    // std::cout << "Coords:" << std::chrono::duration_cast<std::chrono::microseconds>(wallStart-coordStart).count() << "µs" << '\n';
    // std::cout << "Walls:" << std::chrono::duration_cast<std::chrono::microseconds>(end-wallStart).count() << "µs" << '\n';
    Mat relativeImg = imread(BASEIMGPATH);

    // #ifdef SHOW_PLINES
    // // Draw the lines
    // for(size_t i = 0; i < linesP.size(); i++)
    // {
    //     Vec4i l = linesP[i];
    //     line( showLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 5, LINE_AA);
    // }
    // #endif

    double orientation = lAnalyser.getOrientation();
    Point relativePosition = lAnalyser.getTilePosition();
    std::array<std::array<cv::Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> relMap = lAnalyser.getMap();

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Full analysis:" << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "µs" << std::endl;

    #ifdef SHOW_CLINES
    for (size_t i = 0; i < combinedLines.size(); i++)
    {
        line(showLines, combinedLines[i][0], combinedLines[i][1], Scalar(0, 0, 255), 5, LINE_AA);
    }
    // for (size_t i = 0; i < renderPoint.size(); i++)
    // {
    //     circle(showLines, renderPoint[i], 7, Scalar(255, 100, 100), 5);
    //     circle(relativeImg, renderPoint[i], 7, Scalar(255, 100, 100), 5);
    // }
    #endif

    // for (size_t i = 0; i < TILE_READ_AMOUNT+1; i++)
    // {
    //     for (size_t j = 0; j < TILE_READ_AMOUNT+1; j++)
    //     {
    //         circle(showLines, transformPoint(Point(tileEndPointsXY[i], tileEndPointsXY[j]) + relativePosition - Point(150, 150), Point(0,0), -orientation) + ORIGIN, 5, Scalar(255, 255, 255), 5);
    //         circle(relativeImg, Point(tileEndPointsXY[i], tileEndPointsXY[j])+ORIGIN, 3, Scalar(255, 255, 255), 3);
    //     }
    // }

    line(showLines, Point(0, ORIGIN_Y), Point(IMG_SIZE[0], ORIGIN_Y), Scalar(200,200,200));
    line(showLines, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE[1]), Scalar(200,200,200));
    line(relativeImg, Point(0, ORIGIN_Y), Point(IMG_SIZE[0], ORIGIN_Y), Scalar(200,200,200));
    line(relativeImg, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE[1]), Scalar(200,200,200));

    for (int i = 0; i < 2000; i += 300)
    {
        line(showLines, transformPoint(Point(0, ORIGIN_Y + relativePosition.y - i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE[0], ORIGIN_Y + relativePosition.y - i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(showLines, transformPoint(Point(ORIGIN_X + relativePosition.x - i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + relativePosition.x - i, IMG_SIZE[1]), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(showLines, transformPoint(Point(0, ORIGIN_Y + relativePosition.y + i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE[0], ORIGIN_Y + relativePosition.y + i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(showLines, transformPoint(Point(ORIGIN_X + relativePosition.x + i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + relativePosition.x + i, IMG_SIZE[1]), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        
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

    circle(showLines, Point(ORIGIN_X, ORIGIN_Y), 5, Scalar(255, 120, 255), 3);
    circle(relativeImg, Point(ORIGIN_X, ORIGIN_Y) - relativePosition + Point(150, 150), 5, Scalar(255, 120, 255), 3);

    #ifdef SHOW_IMG
    string winName = "Combined lines (in red), Hough detected lines (in green)";
    namedWindow(winName, WINDOW_NORMAL);
    imshow(winName, showLines(SHOW_IMG_SIZE, SHOW_IMG_SIZE));
    resizeWindow(winName, Size(1000, 1000));

    string winName2 = "Relative img";
    namedWindow(winName2, WINDOW_NORMAL);
    imshow(winName2, relativeImg(SHOW_IMG_SIZE, SHOW_IMG_SIZE));
    resizeWindow(winName2, Size(1000, 1000));
    waitKey(0);
    #endif

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
    #endif //CODE_ANALYSIS

    return 0;
}

