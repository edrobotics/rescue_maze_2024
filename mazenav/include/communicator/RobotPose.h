#pragma once


// General information about coordinate system:
// Left-handed coordinate system with its origin in the lower left corner of the tile, eg. positive X and Z point towards the current tile.

class RobotPose
{
    public:
    // XYZ Positional coordinates (translation)
    int t_x {0};
    int t_y {0};
    int t_z {0};

    // Euler angles (XZY Rotation order)
    double r_x {0}; 
    double r_y {0}; 
    double r_z {0}; 
};