#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>

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

    static double calcDist2d(Transform tf1, Transform tf2);

    // Return the inverse of the transform
    Transform inverse();

    static bool test();

    // Copy constructor
    
    // Assignment operator
    // Transform operator= (const Transform& t2);
    // Sum of the transforms
    Transform operator+ (Transform& t2);
    // Get arg1 in relation to arg2 (transform from arg2 to arg1)
    Transform operator- (Transform& t2);
    // Returns where you land if you add t2
    Transform operator+= (Transform& t2);
    // Returns where you land if you subtract t2
    Transform operator-= (Transform& t2);
    // Invert the transform
    Transform operator-() const;
    // Check if equal
    bool operator==(const Transform& tf) const;

};

std::ostream& operator << (std::ostream& os, Transform& tf);
