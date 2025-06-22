#pragma once

#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/action_type.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/action_result.hpp"
#include "yaml-cpp/yaml.h"

namespace falcons::action_planning::types
{

typedef mra_common_msgs::msg::WorldState WorldState;
typedef mra_common_msgs::msg::Action ActionInput;
typedef mra_common_msgs::msg::ActionType ActionType;
typedef mra_common_msgs::msg::Targets Targets;
typedef mra_common_msgs::msg::ActionResult ActionResult;
typedef YAML::Node Settings;

}; // end of namespace
