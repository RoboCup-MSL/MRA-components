#pragma once

#include "mra_action_base/ActionBase.hpp"

namespace falcons::action_getball
{

class ActionGetball : public mra::common::action_base::ActionBase
{
public:
    ActionGetball();

    void tick(
        const mra::common::action_base::types::WorldState& world_state,
        const mra::common::action_base::types::Settings& settings,
        mra::common::action_base::types::ActionResult& action_result,
        mra::common::action_base::types::Targets& targets
    ) override;
};

} // namespace falcons::action_getball
