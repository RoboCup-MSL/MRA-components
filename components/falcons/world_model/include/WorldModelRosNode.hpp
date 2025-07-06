#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/vision_objects.hpp"
#include "falcons_msgs/msg/feedback.hpp"
#include "WorldModelTypes.hpp"
#include <memory>

namespace falcons
{

/**
 * ROS2 node wrapper for WorldModel component.
 * Handles ROS2 message subscriptions/publications and delegates to core logic.
 */
class WorldModelRosNode : public rclcpp::Node
{
public:
    WorldModelRosNode();
    ~WorldModelRosNode();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons
