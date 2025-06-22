#pragma once

#include <string>
#include <memory>
#include <type_traits>
#include <cstdint>
#include "Types.hpp"

namespace falcons::action_planning
{

class ActionBase
{
public:
    ActionBase(const std::string& name, uint8_t type)
    : name_(name), type_(type)
    {}

    virtual ~ActionBase() = default;

    // Called once at startup or config reload
    virtual void initialize(const types::Settings& config) { config_ = config; };

    // Called every tick with new input
    virtual void tick(
        const types::WorldState& world_state,
        const types::Settings& input,
        types::ActionResult& action_result,
        types::Targets& targets
    ) = 0;

    // Convenience overload for tick without custom settings
    void tick(const types::WorldState& world_state, types::ActionResult& action_result, types::Targets& targets)
    {
        tick(world_state, config_, action_result, targets);
    }

    // Called at shutdown or when action is finished
    virtual void finalize() {}

    // Getters
    std::string getName() const { return name_; }

protected:
    std::string name_;
    uint8_t type_; // Use ActionType constants
    types::Settings config_; // Static configuration settings for the action
}; // class ActionBase

} // namespace falcons::action_planning
