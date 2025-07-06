#pragma once

#include <memory>
#include <unordered_map>
#include "mra_action_base/ActionBase.hpp"
#include "mra_action_base/ActionTypes.hpp"
#include "Configuration.hpp" // TODO split/rename to ActionConfiguration.hpp

namespace falcons::action_planning
{

class ActionPlanner {

public:
    ActionPlanner(std::unique_ptr<ConfigurationROS> configurator);
    void tick(
        const mra::common::action_base::types::WorldState &world_state,
        const mra::common::action_base::types::ActionInput &action_input,
        mra::common::action_base::types::ActionResult &action_result,
        mra::common::action_base::types::Targets &targets
    );

private:
    std::unique_ptr<ConfigurationROS> configurator_;
    std::unordered_map<uint8_t, std::unique_ptr<mra::common::action_base::ActionBase>> actions_;
    uint8_t prev_action_type_ = mra::common::action_base::types::ActionType::ACTION_INVALID;

}; // class ActionPlanner

} // namespace falcons::action_planning
