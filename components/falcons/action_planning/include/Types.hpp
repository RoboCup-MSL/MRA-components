#pragma once

#include "mra_common_msgs/msg/world_state.hpp"
#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/targets.hpp"
#include "mra_common_msgs/msg/action_result.hpp"

namespace falcons::action_planning::types
{

typedef mra_common_msgs::msg::WorldState WorldState;
typedef mra_common_msgs::msg::Action ActionInput;
typedef mra_common_msgs::msg::Targets Targets;
typedef mra_common_msgs::msg::ActionResult ActionResult;

}; // end of namespace
