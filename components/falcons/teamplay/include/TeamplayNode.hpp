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
     * Process world state input and generate action decision
     * @param world_state Current world state from ROS2 message
     * @return Action decision to be published
     */
    Action processWorldState(const WorldState& world_state);

    /**
     * Reset the teamplay state
     */
    void reset();

private:
    class Implementation;
    std::unique_ptr<Implementation> _impl;
};

} // namespace falcons
