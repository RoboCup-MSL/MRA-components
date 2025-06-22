#pragma once

#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/action_type.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/action_result.hpp"

namespace falcons::action_planning::types
{

typedef mra_common_msgs::msg::WorldState WorldState;
typedef mra_common_msgs::msg::Action ActionInput;
typedef mra_common_msgs::msg::ActionType ActionType;
typedef mra_common_msgs::msg::Targets Targets;
typedef mra_common_msgs::msg::ActionResult ActionResult;
struct Settings
{
    double radius = 2.0; // Default radius, can be overridden by input
    // Add more settings as needed, e.g. speed, timeout, etc.
}; // TODO: #include "yaml-cpp/yaml.h"

}; // end of namespace
