#include "TeamplayNode.hpp"
#include "ActionChoice.hpp"
#include "mra_tracing/tracing.hpp"

namespace falcons
{

class TeamplayNode::Implementation
{
public:
    Implementation()
        : action_choice_()
        , current_world_state_()
        , current_action_()
    {
    }

    void feedWorldState(const WorldState& world_state)
    {
        TRACE_FUNCTION_INPUTS(world_state.id);
        current_world_state_ = world_state;
    }

    void tick()
    {
        TRACE_FUNCTION();

        // For now, we just use the action choice logic
        // TODO: In the future, this could analyze current_world_state_ to make smarter decisions
        current_action_ = action_choice_.getCurrentAction();

        TRACE_FUNCTION_OUTPUTS(current_action_.type);
    }

    Action getAction() const
    {
        return current_action_;
    }

    void reset()
    {
        TRACE_FUNCTION();
        action_choice_.reset();
        current_world_state_ = WorldState();
        current_action_ = Action();
    }

private:
    ActionChoice action_choice_;
    WorldState current_world_state_;
    Action current_action_;
};

TeamplayNode::TeamplayNode()
    : _impl(std::make_unique<Implementation>())
{
}

TeamplayNode::~TeamplayNode() = default;

void TeamplayNode::feedWorldState(const WorldState& world_state)
{
    _impl->feedWorldState(world_state);
}

void TeamplayNode::tick()
{
    _impl->tick();
}

Action TeamplayNode::getAction() const
{
    return _impl->getAction();
}

void TeamplayNode::reset()
{
    _impl->reset();
}

} // namespace falcons
