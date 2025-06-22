#include "mra_tracing/tracing.hpp"
#include "ActionPlanner.hpp"
#include "ActionGetBall.hpp"

using namespace falcons::action_planning;

ActionPlanner::ActionPlanner()
{
    // Register all available actions here
    actions_[types::ActionType::ACTION_GETBALL] = std::make_unique<ActionGetBall>();
    // TODO: add more actions, e.g. ACTION_STOP, ACTION_MOVE, etc.
    prev_action_type_ = types::ActionType::ACTION_INVALID;
}

void ActionPlanner::finish_previous_action(uint8_t /*prev_action_type*/)
{
    // TODO: implement cleanup/finalization for previous action if needed
}

void ActionPlanner::tick(
    const types::WorldState &world_state,
    const types::ActionInput &action_input,
    types::ActionResult &action_result,
    types::Targets &targets
) {
    TRACE_FUNCTION();

    // Check if the action type has changed
    uint8_t current_action_type = action_input.type;
    if (current_action_type != prev_action_type_) {
        finish_previous_action(prev_action_type_);
        prev_action_type_ = current_action_type;
    }

    // Call the tick method of the selected action
    auto it = actions_.find(current_action_type);
    if (it != actions_.end()) {
        auto &action = it->second;
        types::Settings settings; // TODO: fill from configuration defaults + optional action input
        action->tick(world_state, settings, action_result, targets);
    } else {
        // Unknown action: set result to invalid
        action_result.status = types::ActionResult::ACTIONRESULT_INVALID;
        action_result.details = "unknown action type";
    }
}
