#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
#include <ldlidar_driver/ldlidar_driver_linux.h>

#include <iostream>
#include <chrono>
#include <math.h>
#include <thread>

#define PORT "/dev/ttyUSB0" //COM4?
#define BAUDRATE 230400U

#define PROJECTPATH std::string("/home/markfri/code/rescue_maze_2024/lidar/")
#define SCANPATH PROJECTPATH + std::string("img/scans/")
#define BASEIMGPATH PROJECTPATH + std::string("img/base.png")
#define SCAN_BASENAME SCANPATH + string("ldlidar_scan_")

using namespace std;
using namespace ldlidar;
using namespace cv;

void createCoords(Points2D& points);

int main(int argc, char const *argv[])
{
     LDLidarDriverLinuxInterface ldInterface;
    // ldInterface.RegisterGetTimestampFunctional(std::bind(GetSystemTimeStamp));
    bool connected = ldInterface.Connect(LDType::LD_19, PORT, BAUDRATE);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    cout << "connected: " << connected << '\n';
    cout << "starting: " << ldInterface.Start() << "\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    Points2D points;
    // for (int i = 0; i < 10; i++)
    // {
      ldInterface.GetLaserScanData(points);
      cout << points.size() << " - size\n";
      createCoords(points);
      // cout << points[1].angle << " - ang\n";
      // cout << points[1].intensity << " - intens\n";
      // cout << points[1].distance << " - dst\n";
      // cout << points[1].stamp << " - stmp 1\n";
      // cout << points[43].stamp << " - stmp 43\n";
      cout << points[0].x << "," << points[0].y << " - xy\n" << points[0].angle << " - angle\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    
    Mat image = imread(BASEIMGPATH, IMREAD_GRAYSCALE);

    int cols = image.cols;
    int rows = image.rows;
    int pixelVal = 255;

    for (int i = 0; i < points.size(); i++)
    {
      if (points[i].x > -image.cols/2 && points[i].x < image.cols/2 && points[i].y > -image.rows/2 && points[i].y < image.rows/2)
        image.at<uchar>(points[i].y + image.cols/2, points[i].x + image.cols/2) = pixelVal;
    }

    ///////////////////////////////////////////////////////////////////////////

    Mat dst, cdstP;

    // Edge detection
    // dst = image.clone();
    Canny(image, dst, 50, 200);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);
    // cdstP = cdst.clone();

    // // Standard Hough Line Transform
    // vector<Vec2f> lines; // will hold the results of the detection
    // HoughLines(dst, lines, 1, CV_PI/180, 50, 50, 10); // runs the actual detection
    // // Draw the lines
    // cout << "lines: " << lines.size() << endl;
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    // }

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }
    // Show results
    // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    // imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);

    ///////////////////////////////////////////////////////////////////////
    
    string tNow = to_string(time(0));
    imwrite(SCAN_BASENAME + tNow + string(".png"), image);
    // imwrite(SCAN_BASENAME + tNow + string("_SHLT.png"), cdst);
    imwrite(SCAN_BASENAME + tNow + string("_PLT.png"), cdstP);
    
    // String windowName = "scan";
    // namedWindow("scan", WindowFlags::WINDOW_NORMAL);
    // imshow("scan", image);
    // resizeWindow("scan", Size(100, 100));

    std::this_thread::sleep_for(std::chrono::seconds(2));

    cout << "closing: " << ldInterface.Stop() << "\n";
    bool disconnected = ldInterface.Disconnect();
    cout << "\ndisconnected: " << disconnected << '\n';

    // cv::waitKey(0);
    return 0;
}

void createCoords(Points2D& points)
{
	for (auto i = points.begin(); i != points.end(); i++)
	{
		i->x = sin(i->angle*M_PI/180)*i->distance;
		i->y = cos(i->angle*M_PI/180)*i->distance;
	}
}