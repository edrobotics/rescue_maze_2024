#include <lidar/lidarFiles.h>

using namespace std;
using namespace ldlidar;

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

// void writeFile(Points2D& points) //Write points to txt file
// {
//     ofstream oFile(SCANTXT_PATH);

//     if (!oFile)
//     {
//         std::cerr << "What error, no file?" << endl;
//         return;
//     }

//     for (size_t i = 0; i < points.size(); i++)
//     {
//         oFile << points[i].distance << " " << points[i].angle << " " << points[i].intensity << "\n";
//     }

//     oFile.flush();
//     oFile.close();
// }
