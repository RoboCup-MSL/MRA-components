#pragma once

// WorldModelTypes.hpp: Type definitions for WorldModel component
// Using ROS message types directly for easier integration and migration

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <mra_common_msgs/msg/vision_object.hpp>
#include <mra_common_msgs/msg/world_state.hpp>
#include <mra_common_msgs/msg/player.hpp>

namespace falcons
{

using Time = builtin_interfaces::msg::Time;
using Pose = geometry_msgs::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
using VisionObject = mra_common_msgs::msg::VisionObject;
using WorldState = mra_common_msgs::msg::WorldState;
using Player = mra_common_msgs::msg::Player;

} // namespace falcons
