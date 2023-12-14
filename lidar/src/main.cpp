#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
#include <ldlidar_driver/ldlidar_driver_linux.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <math.h>
#include <thread>

// #define CODE_READ_LIDAR

#ifndef CODE_READ_LIDAR
// #define CODE_READ_FILE_TXT
#define CODE_READ_FILE_IMG
#else
#define CODE_SAVE_TXT
#define CODE_SAVE_SCAN
// #define CODE_SAVE_LATEST
#endif
#define CODE_SAVE_SCANPLT

#define PORT "/dev/ttyUSB0" //COM4?
#define BAUDRATE 230400U

#define PROJECTPATH std::string("/home/markfri/code/rescue_maze_2024/lidar/")
#define SCANPATH PROJECTPATH + std::string("img/scans/")
#define BASEIMGPATH PROJECTPATH + std::string("img/base.png")
#define SCAN_BASENAME SCANPATH + std::string("ldlidar_scan_")
#define SCANTXT_PATH SCANPATH + std::string("ldlidar_scan_latest.txt")
#define SCANTXT_BASE SCANPATH + std::string("ldlidar_scan_")

#ifdef CODE_READ_FILE_IMG
#define READ_FILE_IMG_PATH SCAN_BASENAME + std::string("latest.png")
#endif

using namespace std;
using namespace ldlidar;
using namespace cv;

void writeFile(Points2D& points);

void createCoords(Points2D& points);

Points2D pointsFromTxt(string path);

struct SLine;

SLine LineFitRANSAC(
    float t,//distance from main line
    float p,//chance of hitting a valid pair
    float e,//percentage of outliers
    int T,//number of expected minimum inliers 
    std::vector<cv::Point>& nzPoints);

// void LineMod(SLine &&line) // Rvalue reference - use in main program?
// {

// }

int main(int argc, char const *argv[]) //TESTA FÖRSTORING AV PUNKTER??? , expandera mer i riktningen åt senare och tidigare punkter (om de är sorterade efter vinkel i point2d)
{
    #ifndef CODE_READ_FILE_IMG
    #ifdef CODE_READ_LIDAR
    LDLidarDriverLinuxInterface ldInterface;
    if (!ldInterface.Connect(LDType::LD_19, PORT, BAUDRATE))
    {
      cout << "Could not connect to lidar on port " + string(PORT) << endl;
      return -1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ldInterface.EnablePointCloudDataFilter(true);
    // cout << "connected: " << connected << '\n';
    cout << "starting: " << ldInterface.Start() << "\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    Points2D points;
    // for (int i = 0; i < 10; i++)
    // {
    ldInterface.GetLaserScanData(points);

    #elif CODE_READ_FILE_TXT
    Points2D points = pointsFromTXT(SCANTXT_PATH, 0);
    #endif

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

    std::this_thread::sleep_for(std::chrono::seconds(2));

    cout << "closing: " << ldInterface.Stop() << "\n";
    bool disconnected = ldInterface.Disconnect();
    cout << "disconnected: " << disconnected << "\n";
    #else
    Mat image = imread(READ_FILE_IMG_PATH, IMREAD_GRAYSCALE);
    #endif

    ///////////////////////////////////////////////////////////////////////////

    Mat dst, cdstP;

    // Edge detection
    dst = image.clone();
    // Canny(image, dst, 50, 200);
    cv::dilate(dst, dst, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);

    // Probabilistic Line Transform

    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/1800, 0, 80, 50/*, 40, 50, 10 */); // runs the actual detection

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
    #ifdef CODE_SAVE_SCAN
    imwrite(SCAN_BASENAME + tNow + string(".png"), image);
    #endif
    #ifdef CODE_SAVE_SCANPLT
    imwrite(SCAN_BASENAME + tNow + string("_PLT.png"), cdstP);
    #endif
    #ifdef CODE_SAVE_LATEST
    imwrite(SCAN_BASENAME + "latest_PLT.png", cdstP);
    imwrite(SCAN_BASENAME + "latest.png", image);
    #endif
    // imwrite(SCAN_BASENAME + tNow + string("_SHLT.png"), cdst);
    
    #ifdef CODE_SAVE_TXT
    writeFile(points);
    #endif

    return 0;
}

void writeFile(Points2D& points)
{
    uint16_t dist[points.size()];
    float ang[points.size()];

    for (int i = 0; i < points.size(); ++i)
    {
        dist[i] = points[i].distance;
        ang[i] = points[i].angle;
    }

    std::cout << "done 1\n";

    ofstream oFile(SCANTXT_PATH);

    if (!oFile)
    {
        std::cout << "What error\n";
        return;
    }

    std::cout << "done 2\n";

    for (int i = 0; i < points.size(); ++i)
    {
        oFile << dist[i] << " " << ang[i] << "\n";
    }

    oFile.flush();
    oFile.close();
}

void createCoords(Points2D& points)
{
	for (auto i = points.begin(); i != points.end(); i++)
	{
		i->x = sin(i->angle*M_PI/180)*i->distance; //+x is right
		i->y = -cos(i->angle*M_PI/180)*i->distance; //+y is forward
	}
}

Points2D pointsFromTxt(string path)
{
    Points2D points;
    ifstream inFile(SCANTXT_PATH);

    int r;
    float v;
    while (inFile >> r >> v)
    {
        PointData point;
        point.distance = r;
        point.angle = v;
        points.push_back(point);
    }

    return points;
}

struct SLine
{
    SLine():
        numOfValidPoints(0),
        params(-1.f, -1.f, -1.f, -1.f)
    {}
    cv::Vec4f params;//(cos(t), sin(t), X0, Y0)
    int numOfValidPoints;
};

cv::Vec4f TotalLeastSquares(
    std::vector<cv::Point>& nzPoints,
    std::vector<int> ptOnLine)
{
    //if there are enough inliers calculate model
    float x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    float dx2, dy2, dxy;
    float t;
    for( size_t i = 0; i < nzPoints.size(); ++i )
    {
        x += ptOnLine[i] * nzPoints[i].x;
        y += ptOnLine[i] * nzPoints[i].y;
        x2 += ptOnLine[i] * nzPoints[i].x * nzPoints[i].x;
        y2 += ptOnLine[i] * nzPoints[i].y * nzPoints[i].y;
        xy += ptOnLine[i] * nzPoints[i].x * nzPoints[i].y;
        w += ptOnLine[i];
    }

    x /= w;
    y /= w;
    x2 /= w;
    y2 /= w;
    xy /= w;

    //Covariance matrix
    dx2 = x2 - x * x;
    dy2 = y2 - y * y;
    dxy = xy - x * y;

    t = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;
    cv::Vec4f line;
    line[0] = (float) cos( t );
    line[1] = (float) sin( t );

    line[2] = (float) x;
    line[3] = (float) y;

    return line;
}

SLine LineFitRANSAC(
    float t,//distance from main line
    float p,//chance of hitting a valid pair
    float e,//percentage of outliers
    int T,//number of expected minimum inliers 
    std::vector<cv::Point>& nzPoints)
{
    int s = 2;//number of points required by the model
    int N = (int)ceilf(log(1-p)/log(1 - pow(1-e, s)));//number of independent trials

    std::vector<SLine> lineCandidates;
    std::vector<int> ptOnLine(nzPoints.size());//is inlier
    RNG rng((uint64)-1);
    SLine line;
    for (int i = 0; i < N; i++)
    {
        //pick two points
        int idx1 = (int)rng.uniform(0, (int)nzPoints.size());
        int idx2 = (int)rng.uniform(0, (int)nzPoints.size());
        cv::Point p1 = nzPoints[idx1];
        cv::Point p2 = nzPoints[idx2];

        //points too close - discard
        if (cv::norm(p1- p2) < t)
        {
            continue;
        }

        //line equation ->  (y1 - y2)X + (x2 - x1)Y + x1y2 - x2y1 = 0 
        float a = static_cast<float>(p1.y - p2.y);
        float b = static_cast<float>(p2.x - p1.x);
        float c = static_cast<float>(p1.x*p2.y - p2.x*p1.y);
        //normalize them
        float scale = 1.f/sqrt(a*a + b*b);
        a *= scale;
        b *= scale;
        c *= scale;

        //count inliers
        int numOfInliers = 0;
        for (size_t i = 0; i < nzPoints.size(); ++i)
        {
            cv::Point& p0 = nzPoints[i];
            float rho      = abs(a*p0.x + b*p0.y + c);
            bool isInlier  = rho  < t;
            if ( isInlier ) numOfInliers++;
            ptOnLine[i]    = isInlier;
        }

        if ( numOfInliers < T)
        {
            continue;
        }

        line.params = TotalLeastSquares( nzPoints, ptOnLine);
        line.numOfValidPoints = numOfInliers;
        lineCandidates.push_back(line);
    }

    int bestLineIdx = 0;
    int bestLineScore = 0;
    for (size_t i = 0; i < lineCandidates.size(); i++)
    {
        if (lineCandidates[i].numOfValidPoints > bestLineScore)
        {
            bestLineIdx = i;
            bestLineScore = lineCandidates[i].numOfValidPoints;
        }
    }

    if ( lineCandidates.empty() )
    {
        return SLine();
    }
    else
    {
        return lineCandidates[bestLineIdx];
    }
}