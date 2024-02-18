#include <lidarLineMaker.h>

using namespace std;
using namespace cv;

LineMaker::LineMaker(ldlidar::Points2D& points, cv::Mat& debugOut)
{
    vector<Vec4i> linesP; //detection output
    drawLines(points, linesP, debugOut);

    mergeLines(linesP, combinedLines);
}

vector<Vec<Point, 2>> LineMaker::getLines()
{
    return combinedLines;
}

void LineMaker::drawLines(ldlidar::Points2D& points, std::vector<cv::Vec4i>& linesOut, cv::Mat& debugOut)
{
    Mat dst = imread(BASEIMGPATH, IMREAD_GRAYSCALE);

    //Paint all lidar points on image by modifying pixels in the image
    for (size_t i = 0; i < points.size(); i++)
    {
      if (points[i].x > -ORIGIN_X && points[i].x < ORIGIN_X && points[i].y > -ORIGIN_Y && points[i].y < ORIGIN_Y)
        dst.at<uchar>(ORIGIN_Y + points[i].y, ORIGIN_X + points[i].x) = 255;
    }

    cv::dilate(dst, dst, getStructuringElement(MorphShapes::MORPH_ELLIPSE, cv::Size(3, 3)));

    HoughLinesP(dst, linesOut, 5, CV_PI/180, 20, 100, 100);//line detection

    cvtColor(dst, debugOut, COLOR_GRAY2BGR);
}

void LineMaker::mergeLines(vector<Vec4i>& lines, vector<Vec<Point, 2>>& out)
{
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec<Point, 2> l = {Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3])};
        bool merged = false;

        for (size_t j = 0; j < out.size(); j++)
        {
            if (mergeLineSegments(out[j], l, out[j]))
            {
                merged = true;
                // cout << "merged pline " << i << " with cline " << j << '\n';
                break;
            }
            // cout << "f" << j << ":" << out.size() << "::" << merged << "  ";
        }

        if (!merged)
        {
            out.push_back(l);
        }
    }

    bool mergedC = false;
    for(auto i = out.begin(); i != out.end(); i++) //Extra merge check on final lines, should be none but might be
    {
        if (mergedC) {
            mergedC = false;
            if (out.size() != 0) i = out.begin();
        }
        for(auto j = i+1; j != out.end(); j++)
        {
            if (mergeLineSegments(*i, *j, *i))
            {
                cout << "!merged c!" << endl;
                out.erase(j);
                mergedC = true;
                break;
            }
        }
    }
    cout << out.size() << " -clines, plines- " << lines.size() << endl;
}

bool LineMaker::mergeLineSegments(const Vec<Point, 2>& line_i, const Vec<Point, 2>& line_j, Vec<Point, 2>& output, double angleThresh, double distThresh, bool extraLogging)
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
        if (extraLogging) std::cout << " r0 " << orientation_j << ";" << orientation_i << " _ " << ": ";
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
        if (extraLogging) std::cout << " r2 " << norm(Vec2d(min({a_y_g, b_y_g, c_y_g, d_y_g}), max({a_y_g, b_y_g, c_y_g, d_y_g}))) << "£ " <<
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