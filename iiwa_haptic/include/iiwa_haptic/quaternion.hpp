#include <geometry_msgs/Quaternion.h>
#include <cmath>

geometry_msgs::Quaternion conjugation(geometry_msgs::Quaternion q);

geometry_msgs::Quaternion inverse(const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion power(geometry_msgs::Quaternion q, double power);

double abs(const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion operator*(geometry_msgs::Quaternion q, const double& a);

geometry_msgs::Quaternion operator/(geometry_msgs::Quaternion q, double a);

geometry_msgs::Quaternion operator*(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2);