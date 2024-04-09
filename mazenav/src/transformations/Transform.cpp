#include "transformations/Transform.h"

Transform::Transform()
{
    Transform {0, 0, 0, 0, 0, 0};
}

Transform::Transform(double p_x, double p_y, double p_z, double r_x, double r_y, double r_z)
{
    pos_x = p_x;
    pos_y = p_y;
    pos_z = p_z;
    rot_x = r_x;
    rot_y = r_y;
    rot_z = r_z;
}

double Transform::calcDist2d(Transform tf1, Transform tf2)
{
    Transform resTf {tf2-tf1};
    return sqrt(resTf.pos_x*resTf.pos_x + resTf.pos_y*resTf.pos_y);
}

Transform Transform::inverse()
{
    Transform inv {};

    inv.rot_z = -rot_z;

    inv.pos_x = -pos_x*cos(inv.rot_z) - (-pos_y)*sin(inv.rot_z);
    inv.pos_y = -pos_y*cos(inv.rot_z) + (-pos_x)*sin(inv.rot_z);

    return inv;
}

// Transform Transform::operator= (const Transform& t2)
// {
//     pos_x = t2.pos_x;
//     pos_y = t2.pos_y;
//     pos_z = t2.pos_z;

//     rot_x = t2.rot_x;
//     rot_y = t2.rot_y;
//     rot_z = t2.rot_z;

//     return *this;
// }

Transform Transform::operator- (Transform& t2)
{
    return *this-=t2;
}

Transform Transform::operator+ (Transform& t2)
{
    return *this+=t2;

}

Transform Transform::operator+= (Transform& t2)
{
    this->pos_x += t2.pos_x*cos(this->rot_z)-t2.pos_y*sin(this->rot_z);
    this->pos_y += t2.pos_y*cos(this->rot_z)+t2.pos_x*sin(this->rot_z);
    this->pos_z += t2.pos_z;

    this->rot_x += t2.rot_x;
    this->rot_y += t2.rot_y;
    this->rot_z += t2.rot_z;

    return *this;
}

Transform Transform::operator-= (Transform& t2)
{
    Transform res {};
    Transform inv = t2.inverse();

    res = *this + inv;

    this->rot_x = res.rot_x;
    this->rot_y = res.rot_y;
    this->rot_z = res.rot_z;
    this->pos_x = res.pos_x;
    this->pos_y = res.pos_y;
    this->pos_z = res.pos_z;

    return *this;

}

Transform Transform::operator-() const
{
    Transform cur {pos_x, pos_y, pos_z, rot_x, rot_y, rot_z};
    
    return cur.inverse();
}


std::ostream& operator << (std::ostream& os, Transform& tf)
{
    os << "p_x = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.pos_x << "  "
    << "p_y = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.pos_y << "  "
    << "p_z = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.pos_z << "  "

    << "r_x = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.rot_x << "  "
    << "r_y = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.rot_y << "  "
    << "r_z = " << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(1) << tf.rot_z << "  ";

    return os;
}


bool Transform::operator==(const Transform tf) const
{
    // If all of these are equal, return true
    return this->pos_x == tf.pos_x &&
           this->pos_y == tf.pos_y &&
           this->rot_z == tf.rot_z;
}

#include <vector>
bool Transform::test()
{
    Transform t0 {};
    Transform k1 {1, 1, 0, 0, 0, 0};
    Transform k2 {-1, 1, 0, 0, 0, 0};
    Transform k3 {-1, -1, 0, 0, 0, 0};
    Transform k4 {1, -1, 0, 0, 0, 0};

    std::cout << "No rotations" << "\n";

    std::vector<Transform> kvartiler {k1, k2, k3, k4};
    for (auto& tf : kvartiler)
    {
        Transform res {tf-tf};
        std::cout << res << "\n";
    }

    std::cout << "With rotations" << "\n";

    for (auto& tf : kvartiler)
    {
        tf.rot_z = M_PI_4;
        for (int i=0;i<4;++i)
        {
            Transform res {tf-tf};
            std::cout << res << "\n";
            tf.rot_z += M_PI_2;
        }
    }

    std::cout << "All possibilities for inversions tested" << "\n";
    std::cout << "A successful test contains only 0 everywhere" << "\n";

    return true;

}