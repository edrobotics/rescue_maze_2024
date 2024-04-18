#include <lidar/lidarCoordinate.h>

using namespace cv;

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

Point transformPoint(Point p, Point origin, double angle)
{
    int transformedX = static_cast<int>(round((p.y - origin.y) * sin(angle) + (p.x - origin.x) * cos(angle)));
    int transformedY = static_cast<int>(round((p.y - origin.y) * cos(angle) - (p.x - origin.x) * sin(angle)));
    return Point(transformedX, transformedY);
}