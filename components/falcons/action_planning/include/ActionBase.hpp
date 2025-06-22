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
    : name_(name), type_(type), actionresult_(types::ActionResult::ACTIONRESULT_INVALID)
    {}

    virtual ~ActionBase() = default;

    // Called once at startup or config reload
    virtual void initialize(const types::Settings& config) { config_ = config; };

    // Called every tick with new input
    virtual void tick(const types::WorldState& world_state, const types::Settings& input) = 0;
    void tick(const types::WorldState& world_state)
    {
        tick(world_state, config_);
    }

    // Called at shutdown or when action is finished
    virtual void finalize() {}

    // Getters
    std::string getName() const { return name_; }
    uint8_t getType() const { return type_; }
    uint8_t getActionResult() const { return actionresult_; }
    std::string getVerdict() const { return verdict_; }

protected:
    std::string name_;
    uint8_t type_; // Use ActionType constants
    uint8_t actionresult_; // Use ActionResult constants
    std::string verdict_;
    types::Settings config_; // Static configuration settings for the action
}; // class ActionBase

} // namespace falcons::action_planning
