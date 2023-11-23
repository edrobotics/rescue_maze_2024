
// C++ program for the above approach 
#include <iostream> 
#include <opencv2/core/core.hpp> 
  
// Library to include for 
// drawing shapes 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp> 
using namespace cv; 
using namespace std; 
  
// Driver Code 
int main(int argc, char** argv) 
{ 
    // Create a blank image of size 
    // (500 x 500) with black 
    // background (B, G, R) : (0, 0, 0) 
    Mat image(500, 500, CV_8UC3, 
              Scalar(0, 0, 0)); 
  
    // Check if the image is created 
    // successfully 
    if (!image.data) { 
        cout << "Could not open or find"
             << " the image"; 
  
        return 0; 
    } 
  
    Point p1(0, 0), p2(100, 0); 
    Point p3(200, 0), p4(500, 500); 
    int thickness = 2; 
  
    // Line drawn using 8 connected 
    // Bresenham algorithm 
    line(image, p1, p4, Scalar(255, 0, 0), 
         thickness, LINE_8); 
  
    // Line drawn using 4 connected 
    // Bresenham algorithm 
    line(image, p2, p4, Scalar(0, 255, 0), 
         thickness, LINE_4); 
  
    // Antialiased line 
    line(image, p3, p4, Scalar(0, 0, 255), 
         thickness, LINE_AA); 
     
     putText(image,"OpenCV", Point(10,500), FONT_HERSHEY_PLAIN, 4,Scalar(255,255,255),2, LINE_AA);
     circle(image, Point(250, 250), 10, Scalar(0, 0, 255), FILLED);

    // Show our image inside window 
    imshow("Output", image); 
    waitKey(0); 
  
    return 0; 
} 

// #include <opencv2/opencv.hpp>
// // #include <opencv2/imgcodecs.hpp>
// // #include <opencv2/imgproc.hpp>
// // #include <opencv2/highgui.hpp>
// #include <ldlidar_driver/ldlidar_driver_linux.h>

// #define PORT "/dev/ttyUSB0" //COM4?
// // #define BAUDRATE BaudRate::Baud230400;

// #include <iostream>
// #include <chrono>
// #include <math.h>
// #include <thread>

// using namespace std;
// using namespace ldlidar;
// using namespace cv;

// void createCoords(Points2D& points);

// int main(int argc, char const *argv[])
// {
//   // ///////////////////////////////////////////////////////////////////////////

//     // Mat image = imread("img/lines.png");
//     // Mat dst, cdst;

//     // // Edge detection
//     // Canny(image, dst, 50, 200, 3);
//     // // Copy edges to the images that will display the results in BGR
//     // cvtColor(dst, cdst, COLOR_GRAY2BGR);
//     // // Standard Hough Line Transform
//     // vector<Vec2f> lines; // will hold the results of the detection
//     // HoughLines(dst, lines, 1, CV_PI/180, 50); // runs the actual detection
//     // // Draw the lines
//     // cout << "lines: " << lines.size() << endl;
//     // for( size_t i = 0; i < lines.size(); i++ )
//     // {
//     //     float rho = lines[i][0], theta = lines[i][1];
//     //     Point pt1, pt2;
//     //     double a = cos(theta), b = sin(theta);
//     //     double x0 = a*rho, y0 = b*rho;
//     //     pt1.x = cvRound(x0 + 1000*(-b));
//     //     pt1.y = cvRound(y0 + 1000*(a));
//     //     pt2.x = cvRound(x0 - 1000*(-b));
//     //     pt2.y = cvRound(y0 - 1000*(a));
//     //     cout << pt1.x << ", " << pt1.y << " /to/ " << pt2.x << ", " << pt2.y << "\n";
//     //     line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
//     // }

//     // cvtColor(image, cdst, COLOR_GRAY2BGR);
//     // cdst = image.clone();
//     // line( cdst, Point(1, 1), Point(150, 150), Scalar(0,0,255), 30);
//     // imshow("line_in", image);
//     Mat img(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));

//     Point p1(0, 0), p2(100, 0); 
//     Point p3(200, 0), p4(500, 500); 
//     int thickness = 2; 
  
//     // Line drawn using 8 connected 
//     // Bresenham algorithm 
//     line(img, p1, p4, Scalar(255, 0, 0), 
//          thickness, LINE_8); 
//     // line( img, Point(0, 0), Point(150, 150), Scalar(0,0,255), 3, LINE_AA);
//     imshow("line_out", img);
//     waitKey(0);
//     // Show results
//     // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
//     // imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);

//     // ///////////////////////////////////////////////////////////////////////

//     // LDLidarDriverLinuxInterface ldInterface;
//     // // ldInterface.RegisterGetTimestampFunctional(std::bind(GetSystemTimeStamp));
//     // bool connected = ldInterface.Connect(LDType::LD_19, PORT, 230400U);

//     // std::this_thread::sleep_for(std::chrono::seconds(1));

//     // cout << "connected: " << connected << '\n';
//     // cout << "starting: " << ldInterface.Start() << "\n";

//     // std::this_thread::sleep_for(std::chrono::seconds(2));

//     // Points2D points;
//     // // for (int i = 0; i < 10; i++)
//     // // {
//     //   ldInterface.GetLaserScanData(points);
//     //   cout << points.size() << " - size\n";
//     //   createCoords(points);
//     //   // cout << points[1].angle << " - ang\n";
//     //   // cout << points[1].intensity << " - intens\n";
//     //   // cout << points[1].distance << " - dst\n";
//     //   // cout << points[1].stamp << " - stmp 1\n";
//     //   // cout << points[43].stamp << " - stmp 43\n";
//     //   cout << points[0].x << "," << points[0].y << " - xy\n" << points[0].angle << " - angle\n";
//     //   std::this_thread::sleep_for(std::chrono::seconds(1));
//     // // }
    
//     // Mat image = imread("/home/markfri/code/ldlidar/img/test.png", IMREAD_GRAYSCALE);

//     // int cols = image.cols;
//     // int rows = image.rows;
//     // int pixelVal = 255;

//     // for (int i = 0; i < points.size(); i++)
//     // {
//     //   if (points[i].x > -image.cols/2 && points[i].x < image.cols/2 && points[i].y > -image.rows/2 && points[i].y < image.rows/2)
//     //     image.at<uchar>(points[i].y + image.cols/2, points[i].x + image.cols/2) = pixelVal;
//     // }

//     // ///////////////////////////////////////////////////////////////////////////

//     // Mat dst, cdst, cdstP;

//     // // Edge detection
//     // Canny(image, dst, 50, 200, 3);
//     // // Copy edges to the images that will display the results in BGR
//     // cvtColor(dst, cdst, COLOR_GRAY2BGR);
//     // cdstP = cdst.clone();
//     // // Standard Hough Line Transform
//     // vector<Vec2f> lines; // will hold the results of the detection
//     // HoughLines(dst, lines, 1, CV_PI/180, 40); // runs the actual detection
//     // // Draw the lines
//     // cout << "lines: " << lines.size() << endl;
//     // for( size_t i = 0; i < lines.size(); i++ )
//     // {
//     //     float rho = lines[i][0], theta = lines[i][1];
//     //     Point pt1, pt2;
//     //     double a = cos(theta), b = sin(theta);
//     //     double x0 = a*rho, y0 = b*rho;
//     //     pt1.x = cvRound(x0 + 1000*(-b));
//     //     pt1.y = cvRound(y0 + 1000*(a));
//     //     pt2.x = cvRound(x0 - 1000*(-b));
//     //     pt2.y = cvRound(y0 - 1000*(a));
//     //     cout << pt1.x << ", " << pt1.y << " -to- " << pt2.x << ", " << pt2.y << "\n";
//     //     line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
//     // }
//     // // Probabilistic Line Transform
//     // vector<Vec4i> linesP; // will hold the results of the detection
//     // HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
//     // // Draw the lines
//     // for( size_t i = 0; i < linesP.size(); i++ )
//     // {
//     //     Vec4i l = linesP[i];
//     //     line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
//     // }
//     // // Show results
//     // // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
//     // // imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);

//     // ///////////////////////////////////////////////////////////////////////
//     // line( cdst, Point(10,10), Point(500, 500), Scalar(255,255,255), 10);
//     // cdst.at<Scalar>(2, 3) = Scalar(0, 0, 255);
//     // imwrite(string("/home/markfri/code/ldlidar/img/scans/ldlidar_scan_") + to_string(time(0)) + string(".png"), image);
//     // imwrite(string("/home/markfri/code/ldlidar/img/scans/SHLT_ldlidar_scan_") + to_string(time(0)) + string(".png"), cdst);
//     // imwrite(string("/home/markfri/code/ldlidar/img/scans/PLT_ldlidar_scan_") + to_string(time(0)) + string(".png"), cdstP);
    

//     // // String windowName = "scan";
//     // namedWindow("scan", WindowFlags::WINDOW_NORMAL);
//     // imshow("scan", image);
//     // resizeWindow("scan", Size(100, 100));

//     // std::this_thread::sleep_for(std::chrono::seconds(2));

//     // cout << "closing: " << ldInterface.Stop() << "\n";
//     // bool disconnected = ldInterface.Disconnect();
//     // cout << "\ndisconnected: " << disconnected << '\n';

//     // cv::waitKey(0);
//     return 0;
// } //Update LdLidar_driver.h defines for linux

// void createCoords(Points2D& points)
// {
// 	for (auto i = points.begin(); i != points.end(); i++)
// 	{
// 		i->x = sin(i->angle*M_PI/180)*i->distance;
// 		i->y = cos(i->angle*M_PI/180)*i->distance;
// 	}
// }