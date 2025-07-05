#ifndef VELOCITY_CONTROL_CONVERT_HPP
#define VELOCITY_CONTROL_CONVERT_HPP

#include "VelocityControlTypes.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace VelocityControlConvert {

/**
 * Convert ROS2 pose to VelocityControl position
 * @param pose ROS2 geometry_msgs::msg::Pose
 * @return VelocityControlTypes::Position2D
 */
VelocityControlTypes::Position2D convertFromRosPose(const geometry_msgs::msg::Pose& pose);

/**
 * Convert ROS2 twist to VelocityControl velocity
 * @param twist ROS2 geometry_msgs::msg::Twist
 * @return VelocityControlTypes::Velocity2D
 */
VelocityControlTypes::Velocity2D convertFromRosTwist(const geometry_msgs::msg::Twist& twist);

/**
 * Convert VelocityControl velocity to ROS2 twist
 * @param velocity VelocityControlTypes::Velocity2D
 * @return ROS2 geometry_msgs::msg::Twist
 */
geometry_msgs::msg::Twist convertToRosTwist(const VelocityControlTypes::Velocity2D& velocity);

/**
 * Convert VelocityControl position to ROS2 pose
 * @param position VelocityControlTypes::Position2D
 * @return ROS2 geometry_msgs::msg::Pose
 */
geometry_msgs::msg::Pose convertToRosPose(const VelocityControlTypes::Position2D& position);

} // namespace VelocityControlConvert

#endif // VELOCITY_CONTROL_CONVERT_HPP
