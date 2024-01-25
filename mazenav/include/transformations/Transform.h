#pragma once
#include <iostream>
#include <iomanip>

class Transform
{
    public:
    Transform();
    Transform(double p_x, double p_y, double p_z, double r_x, double r_y, double r_z);
    double pos_x {0};
    double pos_y {0};
    double pos_z {0};
    double rot_x {0};
    double rot_y {0};
    double rot_z {0};

    // Sum of the transforms
    Transform operator+ (Transform& t2);
    // Get arg1 in relation to arg2 (transform from arg2 to arg1)
    Transform operator- (Transform& t2);
    Transform operator+= (Transform& t2);
    Transform operator-= (Transform& t2);
};

std::ostream& operator << (std::ostream& os, Transform& tf);
