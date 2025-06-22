#pragma once

#include <memory>
#include <unordered_map>
#include "ActionBase.hpp"
#include "Types.hpp"

namespace falcons::action_planning
{

class ActionPlanner {

public:
    ActionPlanner();
    void tick(
        const types::WorldState &world_state,
        const types::ActionInput &action_input,
        types::ActionResult &action_result,
        types::Targets &targets
    );

private:
    void finish_previous_action(uint8_t prev_action_type);
    std::unordered_map<uint8_t, std::unique_ptr<ActionBase>> actions_;
    uint8_t prev_action_type_ = types::ActionType::ACTION_INVALID;

}; // class ActionPlanner

} // namespace falcons::action_planning
