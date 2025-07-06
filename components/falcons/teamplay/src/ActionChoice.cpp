#include "ActionChoice.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

ActionChoice::ActionChoice()
    : ticks_remaining_(0)
    , current_action_()
{
    current_action_.type = ActionType::ACTION_GETBALL;
}

Action ActionChoice::getCurrentAction()
{
    TRACE_FUNCTION_INPUTS(ticks_remaining_, current_action_.type);

    if (--ticks_remaining_ <= 0)
    {
        updateAction();
        ticks_remaining_ = 10; // Reset the tick counter
    }

    TRACE_FUNCTION_OUTPUTS(ticks_remaining_, current_action_.type);
    return current_action_;
}

void ActionChoice::reset()
{
    TRACE_FUNCTION();
    ticks_remaining_ = 0;
    current_action_.type = ActionType::ACTION_GETBALL;
}

void ActionChoice::updateAction()
{
    // Cycle through a few actions
    if (current_action_.type == ActionType::ACTION_GETBALL)
    {
        current_action_.type = ActionType::ACTION_STOP;
    }
    else if (current_action_.type == ActionType::ACTION_STOP)
    {
        current_action_.type = ActionType::ACTION_MOVE;
    }
    else
    {
        current_action_.type = ActionType::ACTION_GETBALL;
    }
}

} // namespace falcons
