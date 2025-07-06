#pragma once

#include "mra_common_msgs/msg/action.hpp"
#include "mra_common_msgs/msg/action_type.hpp"
#include "mra_common_msgs/msg/world_state.hpp"

namespace falcons
{

// Type aliases for easier use in core logic
using Action = mra_common_msgs::msg::Action;
using ActionType = mra_common_msgs::msg::ActionType;
using WorldState = mra_common_msgs::msg::WorldState;

} // namespace falcons
