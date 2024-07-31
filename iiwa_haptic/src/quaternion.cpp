#include "iiwa_haptic/quaternion.hpp"

geometry_msgs::Quaternion conjugation(geometry_msgs::Quaternion q)
{
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return q;
}

geometry_msgs::Quaternion inverse(const geometry_msgs::Quaternion& q)
{
    return conjugation(q) / abs(q);
}

double abs(const geometry_msgs::Quaternion& q)
{
    return  sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

geometry_msgs::Quaternion log(geometry_msgs::Quaternion q)
{
    double norm_v = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    double theta = atan2(norm_v, q.w);
    double v[3];
    if(norm_v > 0)
    {
        q = q / norm_v;
    }
    q = q * theta;
    q.w = 0;
    return q;
}

geometry_msgs::Quaternion exp(geometry_msgs::Quaternion q)
{
    double norm_v = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    if(norm_v > 0)
    {
        q = q / norm_v;
    }
    q = q * sin(norm_v);
    q.w = cos(norm_v);
    return q;
}

geometry_msgs::Quaternion power(geometry_msgs::Quaternion q, double power)
{
    return exp(log(q) * power);
}

geometry_msgs::Quaternion operator*(geometry_msgs::Quaternion q, const double& a)
{
    q.w *= a;
    q.x *= a;
    q.y *= a;
    q.z *= a;
    return q;
}

geometry_msgs::Quaternion operator/(geometry_msgs::Quaternion q, double a)
{
    a = 1 / a;
    return q * a;
}

geometry_msgs::Quaternion operator*(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2)
{
    geometry_msgs::Quaternion q;
    q.w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
    q.x = (q1.w * q2.x) + (q2.w * q1.x) + (q1.y * q2.z) - (q1.z * q2.y);
    q.y = (q1.w * q2.y) + (q2.w * q1.y) + (q1.z * q2.x) - (q1.x * q2.z);
    q.z = (q1.w * q2.z) + (q2.w * q1.z) + (q1.x * q2.y) - (q1.y * q2.x);
    return q;
}