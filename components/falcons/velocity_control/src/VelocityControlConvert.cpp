#include "VelocityControlConvert.hpp"
#include <cmath>

namespace VelocityControlConvert {

VelocityControlTypes::Position2D convertFromRosPose(const geometry_msgs::msg::Pose& pose)
{
    // Extract x, y from position
    double x = pose.position.x;
    double y = pose.position.y;

    // Convert quaternion to yaw angle (rotation around z-axis)
    // For 2D applications, we only care about yaw
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;
    double qw = pose.orientation.w;

    // Convert quaternion to yaw angle using atan2
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    return VelocityControlTypes::Position2D(x, y, yaw);
}

VelocityControlTypes::Velocity2D convertFromRosTwist(const geometry_msgs::msg::Twist& twist)
{
    // Extract linear x, y and angular z
    double vx = twist.linear.x;
    double vy = twist.linear.y;
    double vz = twist.angular.z;  // Angular velocity around z-axis

    return VelocityControlTypes::Velocity2D(vx, vy, vz);
}

geometry_msgs::msg::Twist convertToRosTwist(const VelocityControlTypes::Velocity2D& velocity)
{
    geometry_msgs::msg::Twist twist;

    // Set linear velocities
    twist.linear.x = velocity.x;
    twist.linear.y = velocity.y;
    twist.linear.z = 0.0;  // No z-linear velocity for 2D motion

    // Set angular velocities
    twist.angular.x = 0.0;  // No rotation around x-axis for 2D motion
    twist.angular.y = 0.0;  // No rotation around y-axis for 2D motion
    twist.angular.z = velocity.rz;

    return twist;
}

geometry_msgs::msg::Pose convertToRosPose(const VelocityControlTypes::Position2D& position)
{
    geometry_msgs::msg::Pose pose;

    // Set position
    pose.position.x = position.x;
    pose.position.y = position.y;
    pose.position.z = 0.0;  // Assume z=0 for 2D motion

    // Convert yaw angle to quaternion
    double yaw = position.rz;
    double half_yaw = yaw * 0.5;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(half_yaw);
    pose.orientation.w = std::cos(half_yaw);

    return pose;
}

} // namespace VelocityControlConvert
