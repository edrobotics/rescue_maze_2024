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

#define IMG_SIZE cv::Vec2i(5000, 5000)
#define IMG_SIZE_X IMG_SIZE[0]
#define IMG_SIZE_Y IMG_SIZE[1]

#define ORIGIN_X IMG_SIZE_X/2
#define ORIGIN_Y IMG_SIZE_Y/2
#define ORIGIN Point(ORIGIN_X, ORIGIN_Y)

#define MODOFFSET_ORIGIN_Y ORIGIN_Y % 300
#define MODOFFSET_ORIGIN_X ORIGIN_X % 300

#define TILE_READ_AMOUNT 9

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
#define SHOW_IMG_SIZE cv::Range(1000, 4000)
#define SHOW_CLINES //Merged
#define SHOW_PLINES //Hough output
#endif

// #define CODE_SAVE_SCANPLT
#endif

#define PORT "/dev/ttyUSB0" //COM4?
#define BAUDRATE 230400U

#ifdef PI
#define PROJECTPATH std::string("/home/theseus/RCJ2024/lidar/")
#else
#define PROJECTPATH std::string("/home/markfri/code/rescue_maze_2024/lidar/")
#endif
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

double piMod(double mod);
double pi2Mod(double mod);
int tileMod(int mod, double modCenter);

class SLine
{
    public:
    SLine(Vec<Point, 2> linePoints)
    {
        if (linePoints[0].x <= linePoints[1].x)
        {
            startPoint = linePoints[0];
            endPoint = linePoints[1];
        }
        else
        {
            startPoint = linePoints[1];
            endPoint = linePoints[0];
        }

        midPoint = (startPoint+endPoint)/2;

        orientation = piMod(atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x)));
        length = norm(endPoint - startPoint);

        //From transformation: distance(P,θ,(x0,y0)) = |cos(θ)(Py-y0)-sin(θ)(Px-xθ)|
        double minDistance = cos(orientation) * (startPoint.y - ORIGIN_Y) - sin(orientation) * (startPoint.x - ORIGIN_X);
        // double minDistanceAbs = abs(cos(orientation) * (startPoint.y - ORIGIN_Y) - sin(orientation) * (startPoint.x - ORIGIN_X));

        //Min dist point transformed x = 0, since it is always the closest point
        // double startXTransformed = (startPoint.y - ORIGIN_Y) * sin(orientation) + (startPoint.x - ORIGIN_X) * cos(orientation);
        // double endXTransformed = (endPoint.y - ORIGIN_Y) * sin(orientation) + (endPoint.x - ORIGIN_X) * cos(orientation);

        // cout << startPoint.x << "; " << startPoint.y << " | " << endPoint.x << "; " << endPoint.y << " at " << orientation*180/M_PI << "⁰ gives min: " << minDistanceAbs << "and x:s " << startXTransformed << "; " << endXTransformed << endl;
        // if (min(startXTransformed, endXTransformed) < 0 && max(startXTransformed, endXTransformed) > 0)
        // {
        //     cout << "picked full: ";
            // distanceFromOrigin = minDistanceAbs;
            closestToOrigin = Point(minDistance * cos(orientation + M_PI/2) + ORIGIN_X, minDistance * sin(orientation + M_PI/2) + ORIGIN_Y);
        //     closestIsMin = true;
        // }
        // else
        // {
        //     cout << "picked end: ";
        //     distanceFromOrigin = min(norm(startPoint - ORIGIN), norm(endPoint - ORIGIN));
        //     double startToOrigin = norm(startPoint - ORIGIN);
        //     double endToOrigin = norm(endPoint - ORIGIN);

        //     if (startToOrigin < endToOrigin)
        //     {
        //         distanceFromOrigin = startToOrigin;
        //         closestToOrigin = startPoint;
        //     }
        //     else
        //     {
        //         distanceFromOrigin = endToOrigin;
        //         closestToOrigin = endPoint;
        //     }
        //     closestIsMin = false;
        // }
        // cout << distanceFromOrigin << endl;
    }
    double orientation;
    double length;

    Point startPoint;
    Point endPoint;
    Point midPoint;

    // double distanceFromOrigin;
    Point closestToOrigin;
    // bool closestIsMin;
};

// enum side
// {
//     front,
//     left,
//     back,
//     right
// };

// void linesToSegments(SLine& line, vector<cv::Vec<cv::Point, 2>>& out);

Point transformPoint(Point p, Point origin, double angle);

Vec<Point, 2> transformLine(SLine line, Point origin, double angle);

// side findSide(Point transformedMidPoint);

void writeFile(Points2D& points);

void createCoords(Points2D& points);

Points2D pointsFromTxt(string path);

bool mergeLineSegments(const Vec<Point, 2>& line_i, const Vec<Point, 2>& line_j, Vec<Point, 2>& output, double angleThresh = M_PI/6, double distThresh = 100, bool extraLogging = false);

// void LineMod(SLine &&line) // Rvalue reference - use in main program?
// {

// }

int main() //TESTA FÖRSTORING AV PUNKTER??? , expandera mer i riktningen åt senare och tidigare punkter (om de är sorterade efter vinkel i point2d)
{//https://stackoverflow.com/questions/59769762/line-detection-in-2d-point-cloud
//https://winwood.matt.how/2015/04/27/processing-lidar.html or ICP
//Read/write intensity in txt
    #ifndef CODE_READ_FILE_IMG
    #ifdef CODE_READ_LIDAR
    //Connecting and starting
    LDLidarDriverLinuxInterface ldInterface;
    if (!ldInterface.Connect(LDType::LD_19, PORT, BAUDRATE))
    {
      cout << "Could not connect to lidar on port " + string(PORT) << endl;
      return -1;
    }

    ldInterface.EnablePointCloudDataFilter(true);
    cout << "starting: " << ldInterface.Start() << "\n";

    std::this_thread::sleep_for(std::chrono::seconds(1));

    Points2D points;
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

    Mat image = imread(BASEIMGPATH, IMREAD_GRAYSCALE);

    //Paint all lidar points on image by modifying pixels
    for (size_t i = 0; i < points.size(); i++)
    {
      if (points[i].x > -ORIGIN_X && points[i].x < ORIGIN_X && points[i].y > -ORIGIN_Y && points[i].y < ORIGIN_Y)
        image.at<uchar>(ORIGIN_Y + points[i].y, ORIGIN_X + points[i].x) = 255;
    }

    #else
    Mat image = imread(READ_FILE_IMG_PATH, IMREAD_GRAYSCALE);
    #endif //CODE_READ_FILE_IMG

    #ifdef CODE_ANALYSIS
    Mat dst, cdstP;

    dst = image.clone();
    cv::dilate(dst, dst, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));
    #ifdef SHOW_IMG
    cvtColor(dst, cdstP, COLOR_GRAY2BGR); //copy to cdstP which displays
    #endif

    // auto begin1 = std::chrono::system_clock::now();
    // auto begin2 = std::chrono::system_clock::now();

    vector<Vec4i> linesP; //detection output
    HoughLinesP(dst, linesP, 5, CV_PI/180, 20, 100, 100);//line detection

    // auto end1 = std::chrono::system_clock::now();


    //**************************************   MERGE    **************************************//

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

    bool mergedC = false;
    for(auto i = combinedLines.begin(); i != combinedLines.end(); i++) //Extra merge check on final lines, should be none but might be
    {
        if (mergedC) {
            mergedC = false;
            if (combinedLines.size() != 0) i = combinedLines.begin();
        }
        for(auto j = i+1; j != combinedLines.end(); j++)
        {
            if (mergeLineSegments(*i, *j, *i))
            {
                cout << "!merged c!" << endl;
                combinedLines.erase(j);
                mergedC = true;
                break;
            }
        }
    }
    cout << combinedLines.size() << " -clines, plines- " << linesP.size() << endl;

    //**************************************   POSE    **************************************//

    vector<SLine> anaLines; //No need for analines so far?
    for (auto i = combinedLines.begin(); i != combinedLines.end(); i++)
    {
        anaLines.push_back(SLine(*i));
    }

    //Find longest line, which we assume is a wall

    size_t longestLineIndex = 0;

    for (size_t i = 0; i < anaLines.size(); i++)
    {
        if (anaLines[i].length > anaLines[longestLineIndex].length)
        {
            longestLineIndex = i;
        }
    }

    //Get the angle of the longest line, other walls should have similar orientations or perpendicular ones.
    double baseAngle = anaLines[longestLineIndex].orientation; //Orientation of the longest line
    double perpAngle = pi2Mod(baseAngle - M_PI/2);             //Perpendiculat line to the longest line

    double baseLineAngDiffSum = 0; //Sum of orientation diff from baseline - from lines that have ~same orientation as baseangle (likely walls)
    int baseLineAmount = 0; //Amount of these lines
    double perpLineAngDiffSum = 0; //Sum of orientation diff from perpendicular to baseline - from lines that have ~same orientation as the perpendicular to baseangle (likely walls)
    int perpLineAmount = 0; //Amount of these lines

    const double wallLengthThreshold = 200; //Minimum length of a line for it to be seen as a wall is 200 mm
    const double angDiffThreshold = M_PI/6; //Minimum angle for a line to be seen as similar to base/perpAngle

    vector<SLine> wallLines; //Found likely walls

    //Add line orientations for likely walls
    for (auto i = anaLines.begin(); i != anaLines.end(); i++)
    {
        if(i->length < wallLengthThreshold) continue;

        double baseDiff = piMod(i->orientation - baseAngle);
        double perpDiff = piMod(i->orientation - perpAngle);

        if (abs(baseDiff) < angDiffThreshold)
        {
            wallLines.push_back(*i);
            baseLineAngDiffSum += baseDiff;
            baseLineAmount++;
            cout << i->orientation*180/M_PI << "⁰ - added as base" << endl;
        }
        else if (abs(perpDiff) < angDiffThreshold)
        {
            wallLines.push_back(*i);
            perpLineAngDiffSum += perpDiff;
            perpLineAmount++;
            cout << i->orientation*180/M_PI << "⁰ - added as perp" << endl;
        }
    }

    //Form orientation averages
    double baseAvg = baseAngle + baseLineAngDiffSum/baseLineAmount;
    double perpAvg = perpAngle + perpLineAngDiffSum/perpLineAmount;
    double orientation;
    if (perpLineAmount > 0)
	{
        orientation = (baseAvg + pi2Mod(perpAvg + M_PI/2))/2;
	}
    else
	{
        orientation = baseAvg;
	}
	orientation = orientation - round(baseAngle*2*M_1_PI)*M_PI_2; //It is now an angle against the x-axis???
    cout << "Orientation: " << orientation*180/M_PI << "⁰; basetot: " << baseAvg*180/M_PI << "⁰; perptot: " << perpAvg*180/M_PI <<
    "⁰, longest: " << anaLines[longestLineIndex].orientation*180/M_PI << endl;

    vector<Point> renderPoint;

    // double leftWallTot = 0;
    // int leftWallAmt = 0;

    // double rightWallTot = 0;
    // int rightWallAmt = 0;

    // double frontWallTot = 0;
    // int frontWallAmt = 0;

    // double backWallTot = 0;
    // int backWallAmt = 0;

    double xPosTot = 0;
    double yPosTot = 0;
    int xPosAmt = 0;
    int yPosAmt = 0;

    // vector<Vec<Point, 2>> allWalls;
    // bool leftWall = false, rightWall = false, frontWall = false, backWall = false;

    // vector<Vec<Point, 2>> xWallLines;
    // vector<Vec<Point, 2>> yWallLines;

    for (auto i = wallLines.begin(); i != wallLines.end(); i++)
    {
        // Vec<Point, 2> snappedLine = transformLine(wallLines[i], ORIGIN , orientation);
        // double angDiff = wallLines[i].orientation;
        // while (angDiff > M_PI/2) angDiff -= M_PI/2;
        // while (angDiff < 0) angDiff += M_PI/2;

        Point midPoint = transformPoint(i->closestToOrigin, ORIGIN, orientation);
        circle(cdstP, midPoint+ORIGIN, 10, Scalar(255, 255, 100), 10);
        cout << "MIDPOS: " << midPoint.x << ", " << midPoint.y << " ,, grad " << i->orientation * 180/M_PI << "->" << abs(piMod(orientation - i->orientation)) * 180/M_PI << flush;

        // int xAway;
        // int yAway;

        // vector<Vec<Point, 2>> segments = linesToSegments(*i);
        // linesToSegments(*i, allWalls);

        double angleDiffAbs = abs(piMod(orientation - i->orientation));
        if (angleDiffAbs >= M_PI_4)
        {
            //LEFTRIGHT
            // xWallLines.push_back({i->startPoint, i->endPoint});
			double currXPosAvg = 0;
			if (xPosAmt != 0)
				currXPosAvg = xPosTot/xPosAmt;

			int xPosPart = tileMod(midPoint.x, currXPosAvg);
            xPosTot += xPosPart;
            xPosAmt++;

            if (midPoint.x > 0)
            {
                //RIGHT SIDE, xpos
                cout << "._.xp" << xPosPart << endl;
            }
            else
            {
                //LEFT SIDE, xneg
                cout << "._.xn" << xPosPart << endl;
            }
        }
        else
        {
            //UPDOWN
            // yWallLines.push_back({i->startPoint, i->endPoint});
			double currYPosAvg = 0;
			if (yPosAmt != 0)
				currYPosAvg = yPosTot/yPosAmt;
			
			int yPosPart = tileMod(midPoint.y, currYPosAvg);
            yPosTot += yPosPart;
            yPosAmt++;

            if (midPoint.y > 0)
            {
                //BACK SIDE, ypos
                cout << "._.yp" << yPosPart << endl;
            }
            else
            {
                //FRONT SIDE, yneg
                cout << "._.yn" << yPosPart << endl;
            }
        }
        //First find updown leftright
        
        // side lSide = findSide(midPoint);

        // switch (lSide)
        // {
        //     case side::front: //y positive
        //         frontWallTot += fmod(midPoint.y + MODOFFSET_ORIGIN_Y, 300);
        //         frontWallAmt++;
        //         break;
        //     case side::left:
        //         leftWallTot += fmod(midPoint.x + MODOFFSET_ORIGIN_X, 300);
        //         leftWallAmt++;
        //         break;
        //     case side::back:
        //         backWallTot += fmod(midPoint.y - MODOFFSET_ORIGIN_Y, 300);
        //         backWallAmt++;
        //         break;
        //     case side::right: // x positive
        //         rightWallTot += fmod(midPoint.x - MODOFFSET_ORIGIN_X, 300);
        //         rightWallAmt++;
        //         break;
        //     default:
        //         break;
        // }

        // if (transformed.x % 300)
        // {

        // }
    }

    // double leftWallAvg = 0;
    // if (leftWallAmt > 0) leftWallAvg = leftWallTot/leftWallAmt;

    // double rightWallAvg = 0;
    // if (rightWallAmt > 0) rightWallAvg = rightWallTot/rightWallAmt;

    // double frontWallAvg = 0;
    // if (frontWallAmt > 0) frontWallAvg = frontWallTot/frontWallAmt;

    // double backWallAvg = 0;
    // if (backWallAmt > 0) backWallAvg = backWallTot/backWallAmt;

    double xPosAvg = 0;
    if (xPosAmt > 0) 
	{
		xPosAvg = xPosTot/xPosAmt;
		if (xPosAvg < 0) xPosAvg+=300;
	}
    else cout << "ERROR NO XPOS CONTRIB" << endl;

    double yPosAvg = 0;
    if (yPosAmt > 0)
	{
		yPosAvg = yPosTot/yPosAmt;
		if (yPosAvg < 0) yPosAvg+=300;
	}
    else cout << "ERROR NO YPOS CONTRIB" << endl;

    // cout << "x " << xPs << ", l " << leftWallAvg << ", b " << backWallAvg << ", r " << rightWallAvg << endl;

    // double xPosRelative = 0;
    // double yPosRelative = 0;
    // double xPosContributors = 0;
    // double yPosContributors = 0;

    // if (leftWallAvg > 0)
    // {
    //     xPosRelative += leftWallAvg;
    //     xPosContributors++;
    // }
    // if (rightWallAvg > 0)
    // {
    //     xPosRelative += 300 - rightWallAvg;
    //     xPosContributors++;
    // }
    // if (frontWallAvg > 0)
    // {
    //     yPosRelative += frontWallAvg;
    //     yPosContributors++;
    // }
    // if (backWallAvg > 0)
    // {
    //     yPosRelative += 300 - backWallAvg;
    //     yPosContributors++;
    // }
    
    // if (xPosAmt != 0)
    //     xPosRelative /= xPosContributors;
    // else
    //     cout << "NO X CONTRIB" << endl;

    // if (yPosContributors != 0)
    //     yPosRelative /= yPosContributors;
    // else
    //     cout << "NO Y CONTRIB" << endl;
    
    // if (xPosContributors == 0 && yPosContributors == 0)
    // {
    //     cout << "ERROR POS ABSOLUTE" << endl;
    // }
    
    cout << "POS: " << xPosAvg << ", " << yPosAvg << endl;

    Point relativePosition = Point(xPosAvg, yPosAvg);
    
    // for (int i = 0; i < allWalls.size(); i++)
    // {
    //     // int x = allWalls[i][0].x;
    //     // int y = allWalls[i][0].y;
    //     // if ()
    //     //if left oriented -> do the stuff {check mod 30 on closest/<any> point in transformed y}
    //     //if updown > other stuff {check mod 30 on closest/<any> point in transformed x}
    //     //Get avareges in both these directions - pehaps weighted with how close and lighter if shorter than 30 (-> remainder from line seg.)
    //     //this gets our position


    //     //void GetMod() {
    //     int modX = allWalls[i][0].x % 30;
    //     int modY = allWalls[i][0].y % 30;
    //     //SWITCH IS LIKELY DONE IN GETMOD, SINCE WALLS CAN BE UPDOWN AND LEFTRIGHT
    //     //{
    //     // enum relOrient
    //     // {
    //     //     leftRight,
    //     //     upDown,
    //     //     both,
    //     //     neither
    //     // };
    //     // relOrient side /*= OrientationCheck(allWalls[i], orientation)*/;

    //     // switch (side)
    //     // {
    //     //     case both:
    //     //         //LRTot += GetModDistanceLR(x) {check mod 30 on closest/<any> point in transformed y}
    //     //         //LRAmount++;
    //     //         //UDTot += GetModDistanceUD(x) {check mod 30 on closest/<any> point in transformed x}
    //     //         //UDAmount++;
    //     //         break;
    //     //     case leftRight:
    //     //         //LRTot += GetModDistanceLR(x) {check mod 30 on closest/<any> point in transformed y}
    //     //         //LRAmount++;
    //     //         break;
    //     //     case upDown:
    //     //         //UDTot += GetModDistanceUD(x) {check mod 30 on closest/<any> point in transformed x}
    //     //         //UDAmount++;
    //     //         break;
    //     //     default:
    //     //         break;
    //     // }
    //     //}
    // }

    //"construct tile system"
    // Get wall locations by seeing how the walls are in relation to the tile system

    Vec<bool, 4> map[TILE_READ_AMOUNT][TILE_READ_AMOUNT] = {{{false, false, false, false}}}; //4,4 is middle - place walls in possible or likely based on general likeliness

    int tileEndPointsXY[TILE_READ_AMOUNT+1];
    for (size_t i = 0; i < TILE_READ_AMOUNT+1; i++)
    {
        tileEndPointsXY[i] = 300 * i - 300*(TILE_READ_AMOUNT)/2;
        // for (size_t j = 0; j < TILE_READ_AMOUNT+1; j++)
        // {
        //     // tileEndPoints[]
        //     tileEndPoints[i][j] = Point(300*i - 300*(TILE_READ_AMOUNT)/2, 300*j - 300*(TILE_READ_AMOUNT)/2);
        // }
    }

    for (auto i = wallLines.begin(); i != wallLines.end(); i++)
    {
        //Split up into l and r lines?
        //Check the closest wall endpoints to line endpoints, draw lines between

        Point startPointRebased = transformPoint(i->startPoint, ORIGIN, orientation) - relativePosition + Point(150, 150); // + OR - ??
        Point endPointRebased = transformPoint(i->endPoint, ORIGIN, orientation) - relativePosition + Point(150, 150); // + OR - ??
        renderPoint.push_back(startPointRebased+ORIGIN);
        renderPoint.push_back(endPointRebased+ORIGIN);

        // int minDiffStartX;
        // int minDiffStartY;
        // int minDiffEndX;
        // int minDiffEndY;
        int minDiffStartXIndex = -1;
        int minDiffStartYIndex = -1;
        int minDiffEndXIndex = -1;
        int minDiffEndYIndex = -1;
        for (size_t j = 0; j < TILE_READ_AMOUNT+1; j++)
        {
            if (abs(tileEndPointsXY[j] - startPointRebased.x) <= 150) //Can be changed to threshold points that are too far
                minDiffStartXIndex = j;
            if (abs(tileEndPointsXY[j] - startPointRebased.y) <= 150)
                minDiffStartYIndex = j;
            if (abs(tileEndPointsXY[j] - endPointRebased.x) <= 150)
                minDiffEndXIndex = j;
            if (abs(tileEndPointsXY[j] - endPointRebased.y) <= 150)
                minDiffEndYIndex = j;
        }

        if (minDiffEndXIndex == -1 || minDiffEndYIndex == -1 || minDiffStartXIndex == -1 || minDiffStartYIndex == -1)
            cout << "wall too far away" << endl;
        else if ((minDiffEndXIndex == minDiffStartXIndex) && (minDiffEndYIndex == minDiffStartYIndex))
        	cout << "too short line" << endl;
        else
        {
			if (minDiffEndXIndex == minDiffStartXIndex)
			{
				int smallerIndex = min(minDiffEndYIndex, minDiffStartYIndex);
				int largerIndex = max(minDiffEndYIndex, minDiffStartYIndex);
				bool setNext = minDiffEndXIndex > 0;
				for (int j = smallerIndex; j < largerIndex; j++)
				{
					map[j][minDiffEndXIndex][1] = true;
					if(setNext) map[j][minDiffEndXIndex-1][3] = true;
				}
			}
			else if (minDiffEndYIndex == minDiffStartYIndex)
			{
				int smallerIndex = min(minDiffEndXIndex, minDiffStartXIndex);
				int largerIndex = max(minDiffEndXIndex, minDiffStartXIndex);
				bool setNext = minDiffEndYIndex > 0;
				for (int j = smallerIndex; j < largerIndex; j++)
				{
					map[minDiffEndYIndex][j][0] = true;
					if(setNext) map[minDiffEndYIndex-1][j][2] = true;
				}
			}
			else
				cout << "line not straight" << endl; //Line was not in one column or row, but multiple
            //Do actual stuff
			//There is a wall between
        }

        //Pre-calculate wall endpoints, then add robot pos to each wall line end point and compare

        //EX::
        
        //      *     * ----*-
        //                 ^
        //      ,          |
        //      *     *     *
        //      | <- only these would count
        //      |
        //      *     *     * <- wall endpoint
        //      
    }

    // for (auto i = yWallLines.begin(); i != yWallLines.end(); i++)
    // {
    //     //copy xWallLines code
    // }

    //foreach wall in walls { tile[getWallMostLikelyPosition(robotPose)] = getWallLeftDownUpRight }

    // cout << leftWall << " - left, " << rightWall << " - right, " << frontWall << " - front, " << backWall << " - back" << endl;

    //iterate through transformed walls and see where they fit



    // auto end2 = std::chrono::system_clock::now();
    // std::cout << "First part:" << std::chrono::duration_cast<std::chrono::milliseconds>(end1-begin1).count() << "ms" << '\n';
    // std::cout << "Full analysis:" << std::chrono::duration_cast<std::chrono::milliseconds>(end2-begin2).count() << "ms" << std::endl;
    // std::cout << "Merge:" << std::chrono::duration_cast<std::chrono::milliseconds>(end2-end1).count() << "ms" << '\n';
    Mat relativeImg = imread(BASEIMGPATH);

    #ifdef SHOW_PLINES
    // Draw the lines
    for(size_t i = 0; i < linesP.size(); i++)
    {
        Vec4i l = linesP[i];
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 5, LINE_AA);
    }
    #endif

    #ifdef SHOW_CLINES
    for (size_t i = 0; i < combinedLines.size(); i++)
    {
        line(cdstP, combinedLines[i][0], combinedLines[i][1], Scalar(0, 0, 255), 5, LINE_AA);
    }
    for (size_t i = 0; i < renderPoint.size(); i++)
    {
        circle(cdstP, renderPoint[i], 7, Scalar(255, 100, 100), 5);
        circle(relativeImg, renderPoint[i], 7, Scalar(255, 100, 100), 5);
    }
    #endif

    for (size_t i = 0; i < TILE_READ_AMOUNT+1; i++)
    {
        for (size_t j = 0; j < TILE_READ_AMOUNT+1; j++)
        {
            circle(cdstP, transformPoint(Point(tileEndPointsXY[i], tileEndPointsXY[j]) + relativePosition - Point(150, 150), Point(0,0), -orientation) + ORIGIN, 5, Scalar(255, 255, 255), 5);
            circle(relativeImg, Point(tileEndPointsXY[i], tileEndPointsXY[j])+ORIGIN, 3, Scalar(255, 255, 255), 3);
        }
    }

    line(cdstP, Point(0, ORIGIN_Y), Point(IMG_SIZE_X, ORIGIN_Y), Scalar(200,200,200));
    line(cdstP, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE_Y), Scalar(200,200,200));
    line(relativeImg, Point(0, ORIGIN_Y), Point(IMG_SIZE_X, ORIGIN_Y), Scalar(200,200,200));
    line(relativeImg, Point(ORIGIN_X, 0), Point(ORIGIN_X, IMG_SIZE_Y), Scalar(200,200,200));

    // line(cdstP, Point(0, ORIGIN_Y - yPosRelative), Point(IMG_SIZE_X, ORIGIN_Y - yPosRelative), Scalar(200,100,200));
    // line(cdstP, Point(ORIGIN_X - xPosRelative, 0), Point(ORIGIN_X - xPosRelative, IMG_SIZE_Y), Scalar(200,100,200));
    // yPosAvg = xPosAvg = 0;
    for (int i = 0; i < 2000; i += 300)
    {
        line(cdstP, transformPoint(Point(0, ORIGIN_Y + yPosAvg - i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE_X, ORIGIN_Y + yPosAvg - i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(cdstP, transformPoint(Point(ORIGIN_X + xPosAvg - i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + xPosAvg - i, IMG_SIZE_Y), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(cdstP, transformPoint(Point(0, ORIGIN_Y + yPosAvg + i), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(IMG_SIZE_X, ORIGIN_Y + yPosAvg + i), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        line(cdstP, transformPoint(Point(ORIGIN_X + xPosAvg + i, 0), ORIGIN, -orientation)+ORIGIN, transformPoint(Point(ORIGIN_X + xPosAvg + i, IMG_SIZE_Y), ORIGIN, -orientation)+ORIGIN, Scalar(200,100,200));
        
        line(relativeImg, Point(0, ORIGIN_Y - 150 - i), Point(IMG_SIZE_X, ORIGIN_Y - 150 - i), Scalar(200,100,200));
        line(relativeImg, Point(ORIGIN_X - 150 - i, 0), Point(ORIGIN_X - 150 - i, IMG_SIZE_Y), Scalar(200,100,200));
        line(relativeImg, Point(0, ORIGIN_Y + 150 + i), Point(IMG_SIZE_X, ORIGIN_Y + 150 + i), Scalar(200,100,200));
        line(relativeImg, Point(ORIGIN_X + 150 + i, 0), Point(ORIGIN_X + 150 + i, IMG_SIZE_Y), Scalar(200,100,200));
    }

	for (size_t i = 0; i < TILE_READ_AMOUNT; i++)
	{
		for (size_t j = 0; j < TILE_READ_AMOUNT; j++)
		{
			if (map[j][i][0])
				line(relativeImg, Point(i*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (map[j][i][1])
				line(relativeImg, Point(i*300 - 300*TILE_READ_AMOUNT/2, j*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point(i*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (map[j][i][2])
				line(relativeImg, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1-1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
			if (map[j][i][3])
				line(relativeImg, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Point((i+1)*300 - 300*TILE_READ_AMOUNT/2, (j+1-1)*300 - 300*TILE_READ_AMOUNT/2)+ORIGIN, Scalar(255, 255, 255), 3);
		}
	}

    // for (size_t i = 0; i < allWalls.size(); i++)
    // {
    //     line(cdstP, allWalls[i][0], allWalls[i][1], Scalar(255, 255, 0), 5, LINE_AA);
    //     cout << "Allwalls " << i << "; 1: " << allWalls[i][0].x << ", " << allWalls[i][0].y << "; 2: " << allWalls[i][1].x << "," << allWalls[i][0].x << ";; l= " << norm(allWalls[i][0] - allWalls[i][1]) << endl;
    // }
    // cout << allWalls.size() << endl;

    circle(cdstP, Point(ORIGIN_X, ORIGIN_Y), 5, Scalar(255, 120, 255), 3);
    circle(relativeImg, Point(ORIGIN_X, ORIGIN_Y) - relativePosition + Point(150, 150), 5, Scalar(255, 120, 255), 3);

    #ifdef SHOW_IMG
    string winName = "Combined lines (in red), Hough detected lines (in green)";
    namedWindow(winName, WINDOW_NORMAL);
    imshow(winName, cdstP(SHOW_IMG_SIZE, SHOW_IMG_SIZE));
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
    imwrite(SCAN_BASENAME + tNow + string("_PLT.png"), cdstP);
    #endif
    #ifdef CODE_SAVE_LATEST
    imwrite(SCAN_BASENAME + "latest_PLT.png", cdstP);
    imwrite(SCAN_BASENAME + "latest.png", image);
    #endif

    #ifdef CODE_SAVE_TXT
    writeFile(points);
    #endif
    #endif //CODE_ANALYSIS

    return 0;
}

Point transformPoint(Point p, Point origin, double angle)
{
    int transformedX = static_cast<int>(round((p.y - origin.y) * sin(angle) + (p.x - origin.x) * cos(angle)));
    int transformedY = static_cast<int>(round((p.y - origin.y) * cos(angle) - (p.x - origin.x) * sin(angle)));
    return Point(transformedX, transformedY);
}

Vec<Point, 2> transformLine(SLine line, Point origin, double angle)
{
    // Point midPoint = transformPoint(line.midPoint, origin, angle);
    // int halfLength = round(line.length/2);
    Point startPoint = transformPoint(line.startPoint, origin, angle);
    Point endPoint = transformPoint(line.endPoint, origin, angle);
    return {startPoint, endPoint};
    // Point endPoint = transformPoint(Point(midPoint.x + halfLength, midPoint.y), midPoint, angle);
    // Point startPoint =
    // return {midPoint};
}

// side findSide(Point transformedMidPoint)
// {
//     transformedMidPoint -= ORIGIN;
//     if (abs(transformedMidPoint.x) > abs(transformedMidPoint.y))
//     {
//         cout << transformedMidPoint.x << endl;
//         if (transformedMidPoint.x > 0)
//         {
//             return side::right;
//         }
//         else
//         {
//             cout << "l\n";
//             return side::left;
//         }
//     }
//     else
//     {
//         if (transformedMidPoint.y > 0)
//         {
//             return side::front;
//         }
//         else
//         {
//             return side::back;
//         }
//     }
// }

// void linesToSegments(SLine& line, vector<cv::Vec<cv::Point, 2>>& out)
// {
//     // if (fmod(line.length, 30) > lineLengthDivergence * round(line.length/30))
//     // {
//     // }

//     int direction;

//     //Get closest line config <if low is closer, remainder is in high>. Basically - find endpoints and go from closer to further
//     double remainingLength = line.length;

//     Point start;
//     Point end;
//     // int segmentNum = 1;

//     if (norm(line.endPoint) > norm(line.startPoint))
//     {
//         end = line.startPoint;
//         //if(maybeRampDirection) { putRemainderInThatDirection }
//         //get points
//         // direction = 1;
//         for (int i = 1; line.length - i*300 >= 0; i++)
//         {
//             start = end;
//             // Point start = transformPoint(Point(i*30,0), Point(0,0), line.orientation);
//             Point end = start + transformPoint(Point(300, 0), Point(0,0), -line.orientation);
//             out.push_back({start, end});
//             // for (int i = 0; i < static_cast<int>(floor(fmod(line.length, 300))); i++)
//             // {
//             // }
//             // segmentNum++;
//         }
//     }
//     else
//     {
//         end = line.endPoint;
//         for (int i = 1; line.length - i*300 >= 0; i++)
//         {
//             start = end;
//             Point end = start + transformPoint(Point(300, 0), Point(0,0), -line.orientation);
//             out.push_back({start, end});
//         }
//     }

//     if (remainingLength >= 200)
//     {
//         start = end;
//         end = line.endPoint;
//         out.push_back({start+ORIGIN, end+ORIGIN});
//     }

//     //Get line configuration 1 (from low to high, remainder in high)
//     //Get line configuration 2 (from high to low, remainder in low)
// }

void writeFile(Points2D& points) //Write points to txt file
{
    ofstream oFile(SCANTXT_PATH);

    if (!oFile)
    {
        std::cerr << "What error, no file?" << endl;
        return;
    }

    for (size_t i = 0; i < points.size(); i++)
    {
        oFile << points[i].distance << " " << points[i].angle << /*" " << points[i].intensity <<*/ "\n";
    }

    oFile.flush();
    oFile.close();
}

void createCoords(Points2D& points) //Convert "right hand" polar to cartesian
{
	for (auto i = points.begin(); i != points.end(); i++)
	{
        double dist = i->distance;
        i->distance *= (pow(dist, 1.5)+450)/pow(dist+10, 1.5); //Point offset function
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
    ifstream inFile(path);

    uint16_t r;
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

/// @brief Takes "modulo" of a value and pi, but the closest absolute instead of staying on the original side of zero - i.e., a positive value can become negative if it becomes closer to zero
/// @param mod The value to take the piMod of
double piMod(double mod)
{
    while (mod < -M_PI/2) mod += M_PI;
    while (mod > M_PI/2) mod -= M_PI;
    return mod;
}

/// @brief Takes "modulo" of a value and 2 pi, but the closest absolute instead of staying on the original side of zero - i.e., a positive value can become negative if it becomes closer to zero
/// @param mod The value to take the pi2Mod of
double pi2Mod(double mod)
{
    while (mod < -M_PI) mod += 2*M_PI;
    while (mod > M_PI) mod -= 2*M_PI;
    return mod;
}

/// @brief Takes "modulo" of a value and tile (300mm), but the closest absolute instead of staying on the original side of zero - i.e., a positive value can become negative if it becomes closer to modCenter
/// @param mod The value to take the tileMod of
/// @param modCenter The value to mod "around"
int tileMod(int mod, double modCenter)
{
    while (mod < modCenter-150) mod += 300;
    while (mod > modCenter+150) mod -= 300;
    return mod;
}

/// @brief Merges lines if they are within thresholds
/// @param line_i First line for merging
/// @param line_j Second line for merging
/// @param output Output vector
/// @param angleThresh The maximum angle difference between the lines that should be merged
/// @param distThresh The maximum gap between lines that should be merged
/// @param extraLogging Whether extra logging should be used
/// @return True if merged, false if not. Should be false normally.
bool mergeLineSegments(const Vec<Point, 2>& line_i, const Vec<Point, 2>& line_j, Vec<Point, 2>& output, double angleThresh, double distThresh, bool extraLogging)
{
    //Get line length
    double line_i_length = norm(line_i[1] - line_i[0]);
    double line_j_length = norm(line_j[1] - line_j[0]);

    //Get orientation (radians), mod with Pi since they are straight lines, to eliminate angle overflow
    double orientation_i = piMod(atan2((line_i[0].y - line_i[1].y), (line_i[0].x - line_i[1].x)));
    double orientation_j = piMod(atan2((line_j[0].y - line_j[1].y), (line_j[0].x - line_j[1].x)));
    double orientation_r = M_PI;

    //Get orientation difference, if it is larger than PI/2 the difference is actually smaller
    double orientationDiff = abs(orientation_i - orientation_j);
    while (orientationDiff > M_PI/2) orientationDiff -= M_PI;

    //Reject lines with too large difference in orientation
    if (abs(orientationDiff) > angleThresh){
        if (extraLogging) cout << " r0 " << orientation_j << ";" << orientation_i << " _ " << ": ";
        return 0;
    }

    //Centroid coords
    double Xg = (line_i_length * (line_i[0].x + line_i[1].x) + line_j_length * (line_j[0].x + line_j[1].x)) /
                (2 * (line_i_length + line_j_length));

    double Yg = (line_i_length * (line_i[0].y + line_i[1].y) + line_j_length * (line_j[0].y + line_j[1].y)) /
                (2 * (line_i_length + line_j_length));

    //Combined orientation
    if (abs(orientation_i - orientation_j) <= M_PI / 2) {
        orientation_r = line_i_length * orientation_i + line_j_length * orientation_j;
        orientation_r /= line_i_length + line_j_length;
    } else {
        orientation_r = line_i_length * orientation_i +
                        line_j_length * (orientation_j - M_PI * orientation_j / abs(orientation_j));
        orientation_r /= line_i_length + line_j_length;
    }

    //Coordinate transformation of points
    double a_x_g = (line_i[0].y - Yg) * sin(orientation_r) + (line_i[0].x - Xg) * cos(orientation_r);
    double a_y_g = (line_i[0].y - Yg) * cos(orientation_r) - (line_i[0].x - Xg) * sin(orientation_r);

    double b_x_g = (line_i[1].y - Yg) * sin(orientation_r) + (line_i[1].x - Xg) * cos(orientation_r);
    double b_y_g = (line_i[1].y - Yg) * cos(orientation_r) - (line_i[1].x - Xg) * sin(orientation_r);

    double c_x_g = (line_j[0].y - Yg) * sin(orientation_r) + (line_j[0].x - Xg) * cos(orientation_r);
    double c_y_g = (line_j[0].y - Yg) * cos(orientation_r) - (line_j[0].x - Xg) * sin(orientation_r);

    double d_x_g = (line_j[1].y - Yg) * sin(orientation_r) + (line_j[1].x - Xg) * cos(orientation_r);
    double d_y_g = (line_j[1].y - Yg) * cos(orientation_r) - (line_j[1].x - Xg) * sin(orientation_r);

    //Line distance relative
    double line_i_rel_length = norm(Vec2d(b_x_g - a_x_g, b_y_g - a_y_g));
    double line_j_rel_length = norm(Vec2d(d_x_g - c_x_g, d_y_g - c_y_g));

    //Orthogonal projections over X axis
    double start_f = min({a_x_g, b_x_g, c_x_g, d_x_g});
    double end_f = max({a_x_g, b_x_g, c_x_g, d_x_g});
    double length_f = norm(Vec2d(end_f - start_f, 0 - 0));

    //Reject lines with too large of a gap in X and too large of a diff between max and min relative Y
    if (length_f - (line_i_rel_length + line_j_rel_length) > distThresh ||
        norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) > distThresh)
    {
        if (extraLogging) cout << " r2 " << norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) << "£ " <<
                            min({a_y_g, b_y_g, c_y_g, d_y_g}) << ":,:" << max({a_y_g, b_y_g, c_y_g, d_y_g}) << "\n";
        return 0;
    }

    //Get start and end points
    int start_x = static_cast<int>(Xg + start_f * cos(orientation_r));
    int start_y = static_cast<int>(Yg + start_f * sin(orientation_r));
    int end_x = static_cast<int>(Xg + end_f * cos(orientation_r));
    int end_y = static_cast<int>(Yg + end_f * sin(orientation_r));

        // cout << "distance between lines: " << norm(line_i[0] - line_j[0]) << endl;
        // cout << "real lines angle: " << orientation_i * 180 / M_PI << ", " << orientation_j * 180 / M_PI << endl;
        // cout << "orientation angle: " << orientation_r * 180 / M_PI << endl;
        // cout << "centroids: " << Xg << ", " << Yg << endl;
        // cout << "relative lines length: " << line_i_rel_length << ", " << line_j_rel_length << endl;
        // cout << "real lines length: " << line_i_length << ", " << line_j_length << endl;
        // cout << "final line length: " << length_f << endl;
        // cout << "final line endpoints: (" << start_x << ", " << start_y << "), (" << end_x << ", " << end_y << ")" << endl;

    if (extraLogging){ //Show the merge as an image, transformed and not
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

    //Return final line
    output = {Point(start_x, start_y), Point(end_x, end_y)};
    return 1;
}
