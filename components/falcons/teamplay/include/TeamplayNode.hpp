#pragma once

#include "TeamplayTypes.hpp"
#include <memory>

namespace falcons
{

/**
 * TeamplayNode implements the core teamplay functionality.
 * It handles action planning and decision making based on world state.
 */
class TeamplayNode
{
public:
    TeamplayNode();
    ~TeamplayNode();

    /**
     * Feed world state input to the teamplay system
     * @param world_state Current world state from ROS2 message
     */
    void feedWorldState(const WorldState& world_state);

    /**
     * Execute one tick of the teamplay logic
     */
    void tick();

    /**
     * Get the current action decision
     * @return Action decision to be published
     */
    Action getAction() const;

    /**
     * Reset the teamplay state
     */
    void reset();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons
