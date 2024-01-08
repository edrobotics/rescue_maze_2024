#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>

// #include <pcl/console/parse.h>
// #include <pcl/point_cloud.h> // for PointCloud
// #include <pcl/common/io.h> // for copyPointCloud
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/sample_consensus/sac_model_sphere.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/2d/convolution.h>

#include <ldlidar_driver/ldlidar_driver_linux.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <math.h>
#include <thread>

// #define CODE_READ_LIDAR

#ifndef CODE_READ_LIDAR
#define CODE_READ_FILE_TXT
// #define CODE_READ_FILE_IMG
#else
#define CODE_SAVE_TXT
#define CODE_SAVE_SCAN
// #define CODE_SAVE_LATEST
#endif

#define CODE_ANALYSIS

#ifdef CODE_ANALYSIS
#define SHOW_IMG

#ifdef SHOW_IMG
#define SHOW_CLINES
// #define SHOW_PLINES
#endif

// #define CODE_SAVE_SCANPLT
#endif

#define PORT "/dev/ttyUSB0" //COM4?
#define BAUDRATE 230400U

#define PROJECTPATH std::string("/home/markfri/code/rescue_maze_2024/lidar/")
#define SCANPATH PROJECTPATH + std::string("img/scans/")
#define BASEIMGPATH PROJECTPATH + std::string("img/base.png")
#define SCAN_BASENAME SCANPATH + std::string("ldlidar_scan_")
#define SCAN_EXAMPLE_CURRENT PROJECTPATH + std::string("img/example/ldlidar_scan_09")
#define SCANTXT_PATH SCAN_EXAMPLE_CURRENT + std::string(".txt") //PROJECTPATH + std::string("img/example/ldlidar_scan_02.txt") //SCANPATH + std::string("ldlidar_scan_latest.txt")

#ifdef CODE_READ_FILE_IMG
#define READ_FILE_IMG_PATH SCAN_EXAMPLE_CURRENT + std::string(".png") //SCAN_BASENAME + std::string("latest.png")
#endif

using namespace std;
using namespace ldlidar;
using namespace cv;
// using namespace pcl;

void writeFile(Points2D& points);

void createCoords(Points2D& points);

Points2D pointsFromTxt(string path);

bool mergeLineSegments(const Vec<Point, 2>& line_i, const Vec<Point, 2>& line_j, Vec<Point, 2>& output, double angleThresh = M_PI/6, double distThresh = 100, bool extraLogging = false);

struct KLine
{
    int k;
    int m;
    
    KLine(int k, int m) : k(this->k), m(this->m){}
    KLine(Point start, Point end)
    {
        k = (start.y - end.y) / (start.x - end.x);
        m = start.y - k*start.x;
    }
};

// void LineMod(SLine &&line) // Rvalue reference - use in main program?
// {

// }

int main(int argc, char const *argv[]) //TESTA FÖRSTORING AV PUNKTER??? , expandera mer i riktningen åt senare och tidigare punkter (om de är sorterade efter vinkel i point2d)
{//https://stackoverflow.com/questions/59769762/line-detection-in-2d-point-cloud
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

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    cout << "closing: " << ldInterface.Stop() << "\n";
    bool disconnected = ldInterface.Disconnect();
    cout << "disconnected: " << disconnected << "\n";

    #elif defined(CODE_READ_FILE_TXT)
    Points2D points = pointsFromTxt(SCANTXT_PATH);
    #endif //CODE_READ_LIDAR

    cout << points.size() << " points\n";
    createCoords(points);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    Mat image = imread(BASEIMGPATH, IMREAD_GRAYSCALE);

    int cols = image.cols;
    int rows = image.rows;
    int pixelVal = 255;

    for (int i = 0; i < points.size(); i++)
    {
      if (points[i].x > -image.cols/2 && points[i].x < image.cols/2 && points[i].y > -image.rows/2 && points[i].y < image.rows/2)
        image.at<uchar>(points[i].y + image.cols/2, points[i].x + image.cols/2) = pixelVal;
    }
    
    // image = image(Range(2000, 3000), Range(2000, 3000));
    // Mat realIm = imread(SCAN_EXAMPLE_CURRENT + ".png", IMREAD_GRAYSCALE);
    // realIm = realIm(Range(2000, 3000), Range(2000, 3000));
    // imshow("img_window_mod", image);
    // imshow("img_window_real", realIm);
    // waitKey(0);

    #else
    Mat image = imread(READ_FILE_IMG_PATH, IMREAD_GRAYSCALE);
    #endif //CODE_READ_FILE_IMG

    #ifdef CODE_ANALYSIS
    ///////////////////////////////////////////////////////////////////////////

    // PointCloud<PointXY> cloud;
    
    // pcl::RandomSampleConsensus rANSAC;
    Mat dst, cdstP;

    // Edge detection
    dst = image.clone();
    // dst = image(Range(2000, 3000), Range(2000, 3000));
    // Canny(image, dst, 50, 200);
    cv::dilate(dst, dst, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);
    // cdstP = dst.clone();
    // Probabilistic Line Transform
    auto begin1 = std::chrono::system_clock::now();
    auto begin2 = std::chrono::system_clock::now();

    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 5, CV_PI/180, 30, 100, 100);//, 80, 50/*, 40, 50, 10 */); // runs the actual detection

    auto end1 = std::chrono::system_clock::now();

    //**************************************************//
    //*******************   MERGE    *******************//
    //**************************************************//

    // double angleThresh = M_PI/4;
    // double distThresh = 100;

    vector<Vec<Point, 2>> combinedLines;

    for (size_t i = 0; i < linesP.size(); i++)
    {
        Vec<Point, 2> l = {Point(linesP[i][0], linesP[i][1]), Point(linesP[i][2], linesP[i][3])};
        bool merged = false;

        for (size_t j = 0; j < combinedLines.size(); j++)
        {
            if (mergeLineSegments(combinedLines[j], l, combinedLines[j]))
            {
                merged = true;
                // cout << "merged pline " << i << " with cline " << j << '\n';
                break;
            }
            // cout << "f" << j << ":" << combinedLines.size() << "::" << merged << "  ";
        }

        if (!merged)
        {
            combinedLines.push_back(l);
        }
    }

    for(auto i = combinedLines.begin(); i != combinedLines.end(); i++)
    {
        for(auto j = i+1; j != combinedLines.end(); j++)
        {
            // cout << "\nn::" << tih << ",," << tah << ":\n";
            if (mergeLineSegments(*i, *j, *i))
            {
                combinedLines.erase(j);
                --i;
                --j;
            }
            // line(cdstP, combinedLines[i][0], combinedLines[i][1], Scalar(0, 0, 255), 5, LINE_AA);
        }
    }
    cout << combinedLines.size() << " -clines, plines- " << linesP.size() << '\n';

    #ifdef SHOW_PLINES
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 5, LINE_AA);
    }
    #endif

    #ifdef SHOW_CLINES
    for( size_t i = 0; i < combinedLines.size(); i++ )
    {
        line(cdstP, combinedLines[i][0], combinedLines[i][1], Scalar(0, 0, 255), 5, LINE_AA);
    }
    #endif

    auto end2 = std::chrono::system_clock::now();
    std::cout << "First part:" << std::chrono::duration_cast<std::chrono::milliseconds>(end1-begin1).count() << "ms" << '\n';
    std::cout << "Full analysis:" << std::chrono::duration_cast<std::chrono::milliseconds>(end2-begin2).count() << "ms" << std::endl;
    std::cout << "Merge:" << std::chrono::duration_cast<std::chrono::milliseconds>(end2-end1).count() << "ms" << '\n';
    // Canny(cdstP, cdstP, 50, 200);
    // vector<Vec4i> linesP2; // will hold the results of the detection
    // HoughLinesP(cdstP, linesP2, 1, CV_PI/180, 100, 100, 10);//, 80, 50/*, 40, 50, 10 */); // runs the actual detection
    
    // morphologyEx(cdstP, cdstP, MorphTypes::MORPH_CLOSE, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));
    // cv::erode(cdstP, cdstP, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));
    // cvtColor(cdstP, cdstP, COLOR_GRAY2BGR);

    // // // Draw the lines
    // for( size_t i = 0; i < linesP2.size(); i++ )
    // {
    //     Vec4i l = linesP2[i];
    //     line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    // }
    // Show results
    // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    #ifdef SHOW_IMG
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP(Range(2000, 3000), Range(2000, 3000)));
    waitKey(0);
    #endif

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
    #endif //CODE_ANALYSIS

    return 0;
}

void writeFile(Points2D& points)
{
    ofstream oFile(SCANTXT_PATH);

    if (!oFile)
    {
        std::cerr << "What error, no file?\n";
        return;
    }

    for (int i = 0; i < points.size(); ++i)
    {
        oFile << points[i].distance << " " << points[i].angle << "\n";
    }

    oFile.flush();
    oFile.close();
}

void createCoords(Points2D& points)
{
	for (auto i = points.begin(); i != points.end(); i++)
	{
        double dist = i->distance;
        i->distance *= (pow(dist, 1.5)+450)/pow(dist+10, 1.5);
        // if (i->distance < 180 && i->distance != 0)
        // {
        // }
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

double modPi(double mod)
{
    while (mod > M_PI) mod -= M_PI;
    while (mod < M_PI) mod += M_PI;
    return mod;
}

bool mergeLineSegments(const Vec<Point, 2>& line_i, const Vec<Point, 2>& line_j, Vec<Point, 2>& output, double angleThresh, double distThresh, bool extraLogging) 
{
    // line distance
    double line_i_length = norm(line_i[1] - line_i[0]);
    double line_j_length = norm(line_j[1] - line_j[0]);

    // orientation
    double orientation_i = modPi(atan2((line_i[0].y - line_i[1].y), (line_i[0].x - line_i[1].x)));
    double orientation_j = modPi(atan2((line_j[0].y - line_j[1].y), (line_j[0].x - line_j[1].x)));
    double orientation_r = M_PI;

    double orientationDiff = abs(orientation_i - orientation_j);
    while (orientationDiff > M_PI/2) orientationDiff -= M_PI;

    if (extraLogging) cout << " r0 " << orientation_j << ";" << orientation_i << " _ " << ": ";
    if (abs(orientationDiff) > angleThresh){
        // if (extraLogging) cout << " r0 " << orientation_j << ";" << orientation_i << " _ " << ": ";
        return 0;
    }

    // centroids
    double Xg = (line_i_length * (line_i[0].x + line_i[1].x) + line_j_length * (line_j[0].x + line_j[1].x)) /
                (2 * (line_i_length + line_j_length));

    double Yg = (line_i_length * (line_i[0].y + line_i[1].y) + line_j_length * (line_j[0].y + line_j[1].y)) /
                (2 * (line_i_length + line_j_length));

    if (abs(orientation_i - orientation_j) <= M_PI / 2) {
        orientation_r = line_i_length * orientation_i + line_j_length * orientation_j;
        orientation_r /= line_i_length + line_j_length;
    } else {
        orientation_r = line_i_length * orientation_i +
                        line_j_length * (orientation_j - M_PI * orientation_j / abs(orientation_j));
        orientation_r /= line_i_length + line_j_length;
    }

    // coordinate transformation
    double a_x_g = (line_i[0].y - Yg) * sin(orientation_r) + (line_i[0].x - Xg) * cos(orientation_r);
    double a_y_g = (line_i[0].y - Yg) * cos(orientation_r) - (line_i[0].x - Xg) * sin(orientation_r);

    double b_x_g = (line_i[1].y - Yg) * sin(orientation_r) + (line_i[1].x - Xg) * cos(orientation_r);
    double b_y_g = (line_i[1].y - Yg) * cos(orientation_r) - (line_i[1].x - Xg) * sin(orientation_r);

    double c_x_g = (line_j[0].y - Yg) * sin(orientation_r) + (line_j[0].x - Xg) * cos(orientation_r);
    double c_y_g = (line_j[0].y - Yg) * cos(orientation_r) - (line_j[0].x - Xg) * sin(orientation_r);

    double d_x_g = (line_j[1].y - Yg) * sin(orientation_r) + (line_j[1].x - Xg) * cos(orientation_r);
    double d_y_g = (line_j[1].y - Yg) * cos(orientation_r) - (line_j[1].x - Xg) * sin(orientation_r);

    // line distance relative
    double line_i_rel_length = norm(Vec2d(b_x_g - a_x_g, b_y_g - a_y_g));
    double line_j_rel_length = norm(Vec2d(d_x_g - c_x_g, d_y_g - c_y_g));

    // orthogonal projections over the axis X
    double start_f = min({a_x_g, b_x_g, c_x_g, d_x_g});
    double end_f = max({a_x_g, b_x_g, c_x_g, d_x_g});
    double length_f = norm(Vec2d(end_f - start_f, 0 - 0));


    if (length_f - (line_i_rel_length + line_j_rel_length) > distThresh/* ||
        norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) > distThresh*/)
    {
        if (extraLogging) cout << " r2 ";
        return 0;
    }

    if (norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) > distThresh)
    {
        if (extraLogging) cout << " r3: " << norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) << "£ " << 
        min({a_y_g, b_y_g, c_y_g, d_y_g}) << ":,:" << max({a_y_g, b_y_g, c_y_g, d_y_g}) << "\n";
        return 0;
    }

    //start_f = line_i_rel_length * cos(orientation_r);
    //end_f = line_j_rel_length * cos(orientation_r);

    int start_x = static_cast<int>(Xg + start_f * cos(orientation_r));
    int start_y = static_cast<int>(Yg + start_f * sin(orientation_r));
    int end_x = static_cast<int>(Xg + end_f * cos(orientation_r));
    int end_y = static_cast<int>(Yg + end_f * sin(orientation_r));

    // log process
        // cout << "distance between lines: " << norm(line_i[0] - line_j[0]) << endl;
        // cout << "real lines angle: " << orientation_i * 180 / M_PI << ", " << orientation_j * 180 / M_PI << endl;
        // cout << "orientation angle: " << orientation_r * 180 / M_PI << endl;
        // cout << "centroids: " << Xg << ", " << Yg << endl;
        // cout << "relative lines length: " << line_i_rel_length << ", " << line_j_rel_length << endl;
        // cout << "real lines length: " << line_i_length << ", " << line_j_length << endl;
        // cout << "final line length: " << length_f << endl;
        // cout << "final line endpoints: (" << start_x << ", " << start_y << "), (" << end_x << ", " << end_y << ")" << endl;

        // Create a black image
        if (extraLogging){
        Mat img(3000, 3000, CV_8UC3, Scalar(0, 0, 0));
        Mat imgl(3000, 3000, CV_8UC3, Scalar(0, 0, 0));
        namedWindow("Lines");
        namedWindow("LinesL");

        line(img, line_i[0], line_i[1], Scalar(255, 0, 0), 5);
        line(img, line_j[0], line_j[1], Scalar(255, 0, 0), 5);
        circle(img, Point(static_cast<int>(Xg), static_cast<int>(Yg)), 10, Scalar(255, 0, 100), 6);
        line(img, Point(start_x, start_y), Point(end_x, end_y), Scalar(255, 0, 255), 5);
        circle(img, line_i[0], 5, Scalar(0, 0, 255), 5);
        circle(img, line_i[1], 5, Scalar(0, 0, 255), 5);
        circle(img, line_j[0], 5, Scalar(0, 0, 255), 5);
        circle(img, line_j[1], 5, Scalar(0, 0, 255), 5);
        imshow("Lines", img(Range(2000, 3000), Range(2000, 3000)));
        cv::moveWindow("Lines", 0, 0);
        
        circle(imgl, Point(a_x_g+500, a_y_g+500), 5, Scalar(0, 255, 255), 5);
        circle(imgl, Point(b_x_g+500, b_y_g+500), 5, Scalar(0, 255, 255), 5);
        circle(imgl, Point(c_x_g+500, c_y_g+500), 5, Scalar(0, 0, 255), 5);
        circle(imgl, Point(d_x_g+500, d_y_g+500), 5, Scalar(0, 0, 255), 5);
        line(imgl, Point(start_f+500, 500), Point(end_f+500, 500), Scalar(255, 0, 0), 5);
        imshow("LinesL", imgl(Range(0, 1000), Range(0, 1000)));
        waitKey(0);

        destroyAllWindows();
        }
        // imshow("Final Line", img);
        // waitKey(0);
    
    output = {Point(start_x, start_y), Point(end_x, end_y)};
    return 1;
}
