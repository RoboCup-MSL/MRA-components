#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mra_falcons_configuration/ros_config.hpp"
#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/world_state.hpp"
#include "TeamplayTypes.hpp"
#include <memory>

namespace falcons
{

/**
 * ROS2 node wrapper for Teamplay component.
 * Handles ROS2 message subscriptions/publications and delegates to core logic.
 */
class TeamplayRosNode : public rclcpp::Node
{
public:
    TeamplayRosNode();
    ~TeamplayRosNode();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons
