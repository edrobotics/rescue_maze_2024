#include <lidarDataGetter.h>

using namespace ldlidar;
using namespace std;

LidarDataGetter::LidarDataGetter()
{
    #ifdef CODE_READ_LIDAR
    if (!ldInterface.Connect(LDType::LD_19, PORT, BAUDRATE))
    {
      cerr << "Could not connect to lidar on port " + string(PORT) << endl;
      return;
    }

    ldInterface.EnablePointCloudDataFilter(true);
    cout << "starting: " << ldInterface.Start() << "\n";

    lastLidarUseTime = std::chrono::system_clock::now();
    double lidarScanFreq;
    ldInterface.GetLidarScanFreq(lidarScanFreq);
    lidarScanTimeMS = 1000/lidarScanFreq;
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
    createCoords(points);
    lastLidarUseTime = std::chrono::system_clock::now();

    return points;
    #elif defined(CODE_READ_FILE_TXT)
    return pointsFromTxt(SCANTXT_PATH);
    #endif //CODE_READ_LIDAR
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