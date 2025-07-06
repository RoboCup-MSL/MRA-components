#include "TeamplayNode.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

class TeamplayNode::Implementation
{
public:
    Implementation()
        : ticks_remaining_(0)
        , current_action_()
    {
        current_action_.type = ActionType::ACTION_GETBALL;
    }

    Action processWorldState(const WorldState& world_state)
    {
        // MVP implementation: cycle through available actions
        TRACE_FUNCTION_INPUTS(ticks_remaining_, current_action_.type);

        if (--ticks_remaining_ <= 0)
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
            // Reset the tick counter
            ticks_remaining_ = 10;
        }

        TRACE_FUNCTION_OUTPUTS(ticks_remaining_, current_action_.type);
        return current_action_;
    }

    void reset()
    {
        TRACE_FUNCTION();
        ticks_remaining_ = 0;
        current_action_.type = ActionType::ACTION_GETBALL;
    }

private:
    int ticks_remaining_;
    Action current_action_;
};

TeamplayNode::TeamplayNode()
    : _impl(std::make_unique<Implementation>())
{
    TRACE_FUNCTION();
}

TeamplayNode::~TeamplayNode() = default;

Action TeamplayNode::processWorldState(const WorldState& world_state)
{
    return _impl->processWorldState(world_state);
}

void TeamplayNode::reset()
{
    _impl->reset();
}

} // namespace falcons
