#include "lidar/lidarDataGetter.h"

using namespace ldlidar;
using namespace std;

LidarDataGetter::LidarDataGetter()
{
    #ifdef CODE_READ_LIDAR
    if (!ldInterface.Connect(LDType::LD_19, LIDAR_USBPORT, LIDAR_BAUDRATE))
    {
      cerr << "Could not connect to lidar on port " + string(LIDAR_USBPORT) << endl;
      return;
    }

    ldInterface.EnablePointCloudDataFilter(true);
    cout << "starting: " << ldInterface.Start() << "\n";

    lastLidarUseTime = std::chrono::system_clock::now();
    #endif
}

LidarDataGetter::~LidarDataGetter()
{
    #ifdef CODE_READ_LIDAR
    cout << "closing: " << ldInterface.Stop() << "\n";
    bool disconnected = ldInterface.Disconnect();
    cout << "disconnected: " << disconnected << "\n";
    #endif
}

Points2D LidarDataGetter::getData()
{
    #ifdef CODE_READ_LIDAR
    int64_t timeSinceScan = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-lastLidarUseTime).count();
    if (timeSinceScan < lidarScanTimeMS)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeSinceScan - lidarScanTimeMS));
    }

    Points2D points;
    ldInterface.GetLaserScanData(points);
    cout << points.size() << " points\n";
    lastLidarUseTime = std::chrono::system_clock::now();

    return points;
    #elif defined(CODE_READ_FILE_TXT)
    Points2D points = pointsFromTxt(SCANTXT_PATH);
    #endif //CODE_READ_LIDAR
    createCoords(points);
    return points;
}

void LidarDataGetter::createCoords(Points2D& points) //Convert "right hand" polar to cartesian
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