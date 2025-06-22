#pragma once

#include "ActionBase.hpp"
#include <string>

namespace falcons::action_planning
{

class ActionGetBall : public ActionBase
{
public:
    ActionGetBall();
    void tick(const
        types::WorldState& world_state,
        const types::Settings& settings,
        types::ActionResult& action_result,
        types::Targets& targets
    ) override;
};

} // namespace falcons::action_planning
