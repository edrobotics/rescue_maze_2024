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
    this->pos_x += t2.pos_x;
    this->pos_y += t2.pos_y;
    this->pos_z += t2.pos_z;

    this->rot_x += t2.rot_x;
    this->rot_y += t2.rot_y;
    this->rot_z += t2.rot_z;

    return *this;
}

Transform Transform::operator-= (Transform& t2)
{
    this->pos_x -= t2.pos_x;
    this->pos_y -= t2.pos_y;
    this->pos_z -= t2.pos_z;

    this->rot_x -= t2.rot_x;
    this->rot_y -= t2.rot_y;
    this->rot_z -= t2.rot_z;

    return *this;

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