#include "mra_tracing/tracing.hpp"
#include "ActionPlanner.hpp"
#include "ActionGetBall.hpp"
#include <nlohmann/json.hpp>

using namespace falcons::action_planning;

ActionPlanner::ActionPlanner(std::unique_ptr<ConfigurationROS> configurator)
: configurator_(std::move(configurator))
{
    // Register all available actions here
    actions_[types::ActionType::ACTION_GETBALL] = std::make_unique<ActionGetBall>();
    // TODO: add more actions, e.g. ACTION_STOP, ACTION_MOVE, etc.
    prev_action_type_ = types::ActionType::ACTION_INVALID;
}

void ActionPlanner::tick(
    const types::WorldState &world_state,
    const types::ActionInput &action_input,
    types::ActionResult &action_result,
    types::Targets &targets
) {
    TRACE_FUNCTION_INPUTS(world_state, action_input);

    uint8_t current_action_type = action_input.type;
    auto it = actions_.find(current_action_type);

    if (it == actions_.end()) {
        action_result.status = types::ActionResult::ACTIONRESULT_INVALID;
        action_result.details = "unknown action type";
    } else {

        auto &action = it->second;

        // If action type changed, finalize previous and initialize new
        if (current_action_type != prev_action_type_) {
            auto prev_it = actions_.find(prev_action_type_);
            if (prev_it != actions_.end()) {
                prev_it->second->finalize();
            }
            // Take a snapshot of the config at this moment (from YAML + ROS params)
            types::Settings base_config = configurator_->get_scope("action_" + action->getName());
            action->initialize(base_config);
            prev_action_type_ = current_action_type;
        }

        // Prepare config for this tick: start from stored config, apply actionparams as overrule
        types::Settings tick_config = action->mergeConfig(action_input.actionparams);

        // Tick the current action with the tick-local config
        action->tick(world_state, tick_config, action_result, targets);
    }
    TRACE_FUNCTION_OUTPUTS(action_result, targets);
}
